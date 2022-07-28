/*
 * Copyright (C) 2016 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */

#define BTSTACK_FILE__ "a2dp_source_demo.c"

/*
 * a2dp_source_demo.c
 */

// *****************************************************************************
/* EXAMPLE_START(a2dp_source_demo): A2DP Source - Stream Audio and Control Volume
 *
 * @text This A2DP Source example demonstrates how to send an audio data stream 
 * to a remote A2DP Sink device and how to switch between two audio data sources.  
 * In addition, the AVRCP Target is used to answer queries on currently played media,
 * as well as to handle remote playback control, i.e. play, stop, repeat, etc. If HAVE_BTSTACK_STDIN 
 * is set, press SPACE on the console to show the available AVDTP and AVRCP commands.
 *
 * @text To test with a remote device, e.g. a Bluetooth speaker,
 * set the device_addr_string to the Bluetooth address of your 
 * remote device in the code, and use the UI to connect and start playback. 
 * 
 * @text For more info on BTstack audio, see our blog post 
 * [A2DP Sink and Source on STM32 F4 Discovery Board](http://bluekitchen-gmbh.com/a2dp-sink-and-source-on-stm32-f4-discovery-board/).
 * 
 */
// *****************************************************************************


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#include "btstack.h"
#include "hxcmod.h"
#include "mods/mod.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "sdkconfig.h"
#include "esp32/rom/uart.h"
#include "driver/i2s_std.h"
#include "driver/uart.h"

#define $EC                                 ESP_ERROR_CHECK
uint8_t device_id = 0;
char * uart_output_str;
char out_str[256];

#define ECHO_UART_PORT_NUM          	    2

#define LED_PIN                             2

#define I2S_SAMPLE_RATE                     48000
#define I2S_BUFFER_LEN                      1024
#define I2S_BUFFER_COUNT                    32  
#define I2S_PIN_BCK                         26
#define I2S_PIN_LRCK                        25
#define I2S_PIN_DIO                         27

/* Init I2S source for A2DP data */
static void init_i2s(void);

static i2s_chan_handle_t                rx_chan;        // I2S rx channel handler

/* RTOS tasks */
TaskHandle_t                                i2s_task_handle;

/* Create ring buffer */
RingbufHandle_t                             i2s_ringbuf_handle;


#define MAX_DEVICES 10
enum DEVICE_STATE { REMOTE_NAME_REQUEST, REMOTE_NAME_INQUIRED, REMOTE_NAME_FETCHED };
struct device {
    bd_addr_t          address;
    uint8_t            pageScanRepetitionMode;
    uint16_t           clockOffset;
    enum DEVICE_STATE  state; 
    int8_t             rssi;
    char               name[240];
};

#define INQUIRY_INTERVAL 5
struct device devices[MAX_DEVICES];
int deviceCount = 0;

enum STATE {INIT, ACTIVE} ;
enum STATE state = INIT;


static int getDeviceIndexForAddress( bd_addr_t addr){
    int j;
    for (j=0; j< deviceCount; j++){
        if (bd_addr_cmp(addr, devices[j].address) == 0){
            return j;
        }
    }
    return -1;
}

static void start_scan(void){
    sprintf(out_str, "Starting inquiry scan..\n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
    gap_inquiry_start(INQUIRY_INTERVAL);
}

static int has_more_remote_name_requests(void){
    int i;
    for (i=0;i<deviceCount;i++) {
        if (devices[i].state == REMOTE_NAME_REQUEST) return 1;
    }
    return 0;
}

static void do_next_remote_name_request(void){
    int i;
    for (i=0;i<deviceCount;i++) {
        // remote name request
        if (devices[i].state == REMOTE_NAME_REQUEST){
            devices[i].state = REMOTE_NAME_INQUIRED;
            sprintf(out_str, "Get remote name of %s...\n \r", bd_addr_to_str(devices[i].address));
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            gap_remote_name_request( devices[i].address, devices[i].pageScanRepetitionMode,  devices[i].clockOffset | 0x8000);
            return;
        }
    }
}

static void continue_remote_names(void){
    if (has_more_remote_name_requests()){
        do_next_remote_name_request();
        return;
    } 
    start_scan();
}

// logarithmic volume reduction, samples are divided by 2^x
// #define VOLUME_REDUCTION 3

//#define AVRCP_BROWSING_ENABLED

#define NUM_CHANNELS                2
#define BYTES_PER_AUDIO_SAMPLE      (2*NUM_CHANNELS)
#define AUDIO_TIMEOUT_MS            10 
#define TABLE_SIZE_441HZ            100

#define SBC_STORAGE_SIZE            1030

typedef enum {
    STREAM_SINE = 0,
    STREAM_MOD,
    STREAM_PTS_TEST,
    STREAM_I2S
} stream_data_source_t;
    
typedef struct {
    uint16_t a2dp_cid;
    uint8_t  local_seid;
    uint8_t  remote_seid;
    uint8_t  stream_opened;
    uint16_t avrcp_cid;

    uint32_t time_audio_data_sent; // ms
    uint32_t acc_num_missed_samples;
    uint32_t samples_ready;
    btstack_timer_source_t audio_timer;
    uint8_t  streaming;
    int      max_media_payload_size;
    
    uint8_t  sbc_storage[SBC_STORAGE_SIZE];
    uint16_t sbc_storage_count;
    uint8_t  sbc_ready_to_send;

    uint8_t volume;
} a2dp_media_sending_context_t;

static  uint8_t media_sbc_codec_capabilities[] = {
    (AVDTP_SBC_48000 << 4) | AVDTP_SBC_STEREO,
    0xFF,//(AVDTP_SBC_BLOCK_LENGTH_16 << 4) | (AVDTP_SBC_SUBBANDS_8 << 2) | AVDTP_SBC_ALLOCATION_METHOD_LOUDNESS,
    2, 53
}; 

// input signal: pre-computed int16 sine wave, 44100 Hz at 441 Hz
static const int16_t sine_int16_44100[] = {
     0,    2057,    4107,    6140,    8149,   10126,   12062,   13952,   15786,   17557,
 19260,   20886,   22431,   23886,   25247,   26509,   27666,   28714,   29648,   30466,
 31163,   31738,   32187,   32509,   32702,   32767,   32702,   32509,   32187,   31738,
 31163,   30466,   29648,   28714,   27666,   26509,   25247,   23886,   22431,   20886,
 19260,   17557,   15786,   13952,   12062,   10126,    8149,    6140,    4107,    2057,
     0,   -2057,   -4107,   -6140,   -8149,  -10126,  -12062,  -13952,  -15786,  -17557,
-19260,  -20886,  -22431,  -23886,  -25247,  -26509,  -27666,  -28714,  -29648,  -30466,
-31163,  -31738,  -32187,  -32509,  -32702,  -32767,  -32702,  -32509,  -32187,  -31738,
-31163,  -30466,  -29648,  -28714,  -27666,  -26509,  -25247,  -23886,  -22431,  -20886,
-19260,  -17557,  -15786,  -13952,  -12062,  -10126,   -8149,   -6140,   -4107,   -2057,
};

static const int num_samples_sine_int16_44100 = sizeof(sine_int16_44100) / 2;

// input signal: pre-computed int16 sine wave, 48000 Hz at 441 Hz
static const int16_t sine_int16_48000[] = {
     0,    1905,    3804,    5690,    7557,    9398,   11207,   12978,   14706,   16383,
 18006,   19567,   21062,   22486,   23834,   25101,   26283,   27376,   28377,   29282,
 30087,   30791,   31390,   31884,   32269,   32545,   32712,   32767,   32712,   32545,
 32269,   31884,   31390,   30791,   30087,   29282,   28377,   27376,   26283,   25101,
 23834,   22486,   21062,   19567,   18006,   16383,   14706,   12978,   11207,    9398,
  7557,    5690,    3804,    1905,       0,   -1905,   -3804,   -5690,   -7557,   -9398,
-11207,  -12978,  -14706,  -16384,  -18006,  -19567,  -21062,  -22486,  -23834,  -25101,
-26283,  -27376,  -28377,  -29282,  -30087,  -30791,  -31390,  -31884,  -32269,  -32545,
-32712,  -32767,  -32712,  -32545,  -32269,  -31884,  -31390,  -30791,  -30087,  -29282,
-28377,  -27376,  -26283,  -25101,  -23834,  -22486,  -21062,  -19567,  -18006,  -16384,
-14706,  -12978,  -11207,   -9398,   -7557,   -5690,   -3804,   -1905,  };

// silense if no data in I2S
static const int16_t silence_int16_48000[] = {
     0,    0,    0,    0,    0,    0,   0,   0,   0,   0,
     0,    0,    0,    0,    0,    0,   0,   0,   0,   0,
     0,    0,    0,    0,    0,    0,   0,   0,   0,   0,
     0,    0,    0,    0,    0,    0,   0,   0,   0,   0,
     0,    0,    0,    0,    0,    0,   0,   0,   0,   0,
     0,    0,    0,    0,    0,    0,   0,   0,   0,   0,
     0,    0,    0,    0,    0,    0,   0,   0,   0,   0,
     0,    0,    0,    0,    0,    0,   0,   0,   0,   0,
     0,    0,    0,    0,    0,    0,   0,   0,   0,   0,
     0,    0,    0,    0,    0,    0,   0,   0,   0,   0,
     0,    0,    0,    0,    0,    0,   0,   0,   0,   0, };

static const int num_samples_sine_int16_48000 = sizeof(sine_int16_48000) / 2;

typedef struct {
    int reconfigure;
    
    int num_channels;
    int sampling_frequency;
    int block_length;
    int subbands;
    int min_bitpool_value;
    int max_bitpool_value;
    btstack_sbc_channel_mode_t      channel_mode;
    btstack_sbc_allocation_method_t allocation_method;
} media_codec_configuration_sbc_t;

static btstack_packet_callback_registration_t hci_event_callback_registration;

static const char * device_addr_string = "00:00:00:00:00:00";

static bd_addr_t device_addr;

static uint8_t sdp_a2dp_source_service_buffer[150];
static uint8_t sdp_avrcp_target_service_buffer[200];
static uint8_t sdp_avrcp_controller_service_buffer[200];
static uint8_t device_id_sdp_service_buffer[100];

static media_codec_configuration_sbc_t sbc_configuration;
static btstack_sbc_encoder_state_t sbc_encoder_state;

static uint8_t media_sbc_codec_configuration[4];
static a2dp_media_sending_context_t media_tracker;

static stream_data_source_t data_source;

static int sine_phase;
static int current_sample_rate = 48000;
static int new_sample_rate = 48000;

static int hxcmod_initialized;
static modcontext mod_context;
static tracker_buffer_state trkbuf;

/* AVRCP Target context START */

typedef struct {
    uint8_t track_id[8];
    uint32_t song_length_ms;
    avrcp_playback_status_t status;
    uint32_t song_position_ms; // 0xFFFFFFFF if not supported
} avrcp_play_status_info_t;

// python -c "print('a'*512)"
static const char title[] = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

avrcp_track_t tracks[] = {
    {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01}, 1, "Sine", "Generated", "A2DP Source Demo", "monotone", 12345},
    {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02}, 2, "Nao-deceased", "Decease", "A2DP Source Demo", "vivid", 12345},
    {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03}, 3, (char *)title, "Decease", "A2DP Source Demo", "vivid", 12345},
};
int current_track_index;
avrcp_play_status_info_t play_info;

/* AVRCP Target context END */

/* @section Main Application Setup
 *
 * @text The Listing MainConfiguration shows how to setup AD2P Source and AVRCP services. 
 * Besides calling init() method for each service, you'll also need to register several packet handlers:
 * - hci_packet_handler - handles legacy pairing, here by using fixed '0000' pin code.
 * - a2dp_source_packet_handler - handles events on stream connection status (established, released), the media codec configuration, and, the commands on stream itself (open, pause, stopp).
 * - avrcp_packet_handler - receives connect/disconnect event.
 * - avrcp_controller_packet_handler - receives answers for sent AVRCP commands.
 * - avrcp_target_packet_handler - receives AVRCP commands, and registered notifications.
 * - stdin_process - used to trigger AVRCP commands to the A2DP Source device, such are get now playing info, start, stop, volume control. Requires HAVE_BTSTACK_STDIN.
 *
 * @text To announce A2DP Source and AVRCP services, you need to create corresponding
 * SDP records and register them with the SDP service. 
 */

/* LISTING_START(MainConfiguration): Setup Audio Source and AVRCP Target services */
static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void a2dp_source_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t * event, uint16_t event_size);
static void avrcp_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void avrcp_target_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void avrcp_controller_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
#ifdef HAVE_BTSTACK_STDIN
static void stdin_process(char cmd);
#endif

static void init_gpio_led(void);

static void init_gpio_led(void)
{
    gpio_config_t gpio_conf_led;
    gpio_conf_led.intr_type = GPIO_INTR_DISABLE;
	gpio_conf_led.mode = GPIO_MODE_OUTPUT;
	gpio_conf_led.pin_bit_mask = (1ULL<<LED_PIN );
	gpio_conf_led.pull_down_en = 0;
	gpio_conf_led.pull_up_en = 0;
    
    gpio_config(&gpio_conf_led);

    gpio_set_level(LED_PIN, 1);
}


static void a2dp_demo_hexcmod_configure_sample_rate(int sample_rate);

/* @section Bluetooth Logic 
 *
 * @text The Bluetooth logic is implemented as a state machine within the packet
 * handler. In this example, the following states are passed sequentially:
 * INIT, and ACTIVE.
 */ 


static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);

    bd_addr_t addr;
    int i;
    int index;
    
    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event = hci_event_packet_get_type(packet);

    switch(state){ 
        /* @text In INIT, an inquiry  scan is started, and the application transits to 
         * ACTIVE state.
         */
        case INIT:
            switch(event){
                case BTSTACK_EVENT_STATE:
                    if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
                        start_scan();
                        state = ACTIVE;
                    }
                    break;
                default:
                    break;
            }
            break;

        /* @text In ACTIVE, the following events are processed:
         *  - GAP Inquiry result event: BTstack provides a unified inquiry result that contain
         *    Class of Device (CoD), page scan mode, clock offset. RSSI and name (from EIR) are optional.
         *  - Inquiry complete event: the remote name is requested for devices without a fetched 
         *    name. The state of a remote name can be one of the following: 
         *    REMOTE_NAME_REQUEST, REMOTE_NAME_INQUIRED, or REMOTE_NAME_FETCHED.
         *  - Remote name request complete event: the remote name is stored in the table and the 
         *    state is updated to REMOTE_NAME_FETCHED. The query of remote names is continued.
         */
        case ACTIVE:
            switch(event){

                case GAP_EVENT_INQUIRY_RESULT:
                    if (deviceCount >= MAX_DEVICES) break;  // already full
                    gap_event_inquiry_result_get_bd_addr(packet, addr);
                    index = getDeviceIndexForAddress(addr);
                    if (index >= 0) break;   // already in our list

                    memcpy(devices[deviceCount].address, addr, 6);
                    devices[deviceCount].pageScanRepetitionMode = gap_event_inquiry_result_get_page_scan_repetition_mode(packet);
                    devices[deviceCount].clockOffset = gap_event_inquiry_result_get_clock_offset(packet);
                    // print info
                    sprintf(out_str, "Device found: %s ",  bd_addr_to_str(addr));
                    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

                    sprintf(out_str, "with COD: 0x%06x, ", (unsigned int) gap_event_inquiry_result_get_class_of_device(packet));
                    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

                    sprintf(out_str, "pageScan %d, ",      devices[deviceCount].pageScanRepetitionMode);
                    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

                    sprintf(out_str, "clock offset 0x%04x",devices[deviceCount].clockOffset);
                    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

                    if (gap_event_inquiry_result_get_rssi_available(packet)){
                        int8_t rssi = (int8_t) gap_event_inquiry_result_get_rssi(packet);
                        sprintf(out_str, ", rssi %d dBm", rssi);
                        devices[deviceCount].rssi = rssi;
                        uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
                    }
                    if (gap_event_inquiry_result_get_name_available(packet)){
                        char name_buffer[240];
                        int name_len = gap_event_inquiry_result_get_name_len(packet);
                        memcpy(name_buffer, gap_event_inquiry_result_get_name(packet), name_len);
                        name_buffer[name_len] = 0;

                        sprintf(out_str, ", name '%s'", name_buffer);
                        uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
                        memcpy(devices[deviceCount].name, name_buffer, 240);

                        devices[deviceCount].state = REMOTE_NAME_FETCHED;
                    } else {
                        devices[deviceCount].state = REMOTE_NAME_REQUEST;
                    }
                    sprintf(out_str, "\n \r");
                    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

                    deviceCount++;
                    break;

                case GAP_EVENT_INQUIRY_COMPLETE:
                    for (i=0;i<deviceCount;i++) {
                        // retry remote name request
                        if (devices[i].state == REMOTE_NAME_INQUIRED)
                            devices[i].state = REMOTE_NAME_REQUEST;
                    }
                    continue_remote_names();
                    break;

                case HCI_EVENT_REMOTE_NAME_REQUEST_COMPLETE:
                    reverse_bd_addr(&packet[3], addr);
                    index = getDeviceIndexForAddress(addr);
                    if (index >= 0) {
                        if (packet[2] == 0) {
                            sprintf(out_str, "Name: '%s'\n \r", &packet[9]);
                            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

                            devices[index].state = REMOTE_NAME_FETCHED;
                        } else {
                            sprintf(out_str, "Failed to get name: page timeout\n \r");
                            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
                        }
                    }
                    continue_remote_names();
                    break;

                default:
                    break;
            }
            break;
            
        default:
            break;
    }
}

void i2s_task()    
{
	int i2s_read_len = 512;

	while (true) 
    {
		int16_t* i2s_buff = (int16_t*) calloc(i2s_read_len, sizeof(int16_t));

        size_t bytes_read = 0;

        $EC(i2s_channel_read(rx_chan, i2s_buff, i2s_read_len * 2, &bytes_read, portMAX_DELAY));

        if (bytes_read == 0)
            continue;

        /* Send received data to ring buffer */
        xRingbufferSend(i2s_ringbuf_handle, (char *)i2s_buff, i2s_read_len * 2, 0);

        /* Clean up */
        free(i2s_buff);
	}

}


void init_i2s(void)
{
    /* Get the default channel configuration by helper macro.
     * This helper macro is defined in 'i2s_common.h' and shared by all the i2s communication mode.
     * It can help to specify the I2S role, and port id */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_SLAVE);

    /* Allocate a new rx channel and get the handle of this channel */
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan)); 
    
    /* Setting the configurations, the slot configuration and clock configuration can be generated by the macros
     * These two helper macros is defined in 'i2s_std.h' which can only be used in STD mode.
     * They can help to specify the slot and clock configurations for initialization or updating */
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_NUM_26,
            .ws = GPIO_NUM_25,
            .dout = I2S_GPIO_UNUSED,
            .din = GPIO_NUM_27,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    /* Initialize the channel */
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));

    /* Create a bype ring buffer for I2S audio */
	i2s_ringbuf_handle = xRingbufferCreate(8192, RINGBUF_TYPE_BYTEBUF);

    xTaskCreate(i2s_task, "I2S Task", 8192, NULL, 0, &i2s_task_handle);

}



static int a2dp_source_and_avrcp_services_init(void){    

    init_i2s();

    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));

    // Request role change on reconnecting headset to always use them in slave mode
    hci_set_master_slave_policy(0);

    l2cap_init();

#ifdef ENABLE_BLE
    // Initialize LE Security Manager. Needed for cross-transport key derivation
    sm_init();
#endif

    // Initialize  A2DP Source
    a2dp_source_init();
    a2dp_source_register_packet_handler(&a2dp_source_packet_handler);

    // Create stream endpoint
    avdtp_stream_endpoint_t * local_stream_endpoint = a2dp_source_create_stream_endpoint(AVDTP_AUDIO, AVDTP_CODEC_SBC, media_sbc_codec_capabilities, sizeof(media_sbc_codec_capabilities), media_sbc_codec_configuration, sizeof(media_sbc_codec_configuration));
    if (!local_stream_endpoint){
        sprintf(out_str, "A2DP Source: not enough memory to create local stream endpoint\n \r");
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
        return 1;
    }

    // Store stream enpoint's SEP ID, as it is used by A2DP API to indentify the stream endpoint
    media_tracker.local_seid = avdtp_local_seid(local_stream_endpoint);
    avdtp_source_register_delay_reporting_category(media_tracker.local_seid);

    // Initialize AVRCP Service
    avrcp_init();
    avrcp_register_packet_handler(&avrcp_packet_handler);
    // Initialize AVRCP Target
    avrcp_target_init();
    avrcp_target_register_packet_handler(&avrcp_target_packet_handler);

    // Initialize AVRCP Controller
    avrcp_controller_init();
    avrcp_controller_register_packet_handler(&avrcp_controller_packet_handler);

    // Initialize SDP, 
    sdp_init();
    
    // Create A2DP Source service record and register it with SDP
    memset(sdp_a2dp_source_service_buffer, 0, sizeof(sdp_a2dp_source_service_buffer));
    a2dp_source_create_sdp_record(sdp_a2dp_source_service_buffer, 0x10001, AVDTP_SOURCE_FEATURE_MASK_PLAYER, NULL, NULL);
    sdp_register_service(sdp_a2dp_source_service_buffer);
    
    // Create AVRCP Target service record and register it with SDP. We receive Category 1 commands from the headphone, e.g. play/pause
    memset(sdp_avrcp_target_service_buffer, 0, sizeof(sdp_avrcp_target_service_buffer));
    uint16_t supported_features = AVRCP_FEATURE_MASK_CATEGORY_PLAYER_OR_RECORDER;
#ifdef AVRCP_BROWSING_ENABLED
    supported_features |= AVRCP_FEATURE_MASK_BROWSING;
#endif
    avrcp_target_create_sdp_record(sdp_avrcp_target_service_buffer, 0x10002, supported_features, NULL, NULL);
    sdp_register_service(sdp_avrcp_target_service_buffer);

    // Create AVRCP Controller service record and register it with SDP. We send Category 2 commands to the headphone, e.g. volume up/down
    memset(sdp_avrcp_controller_service_buffer, 0, sizeof(sdp_avrcp_controller_service_buffer));
    uint16_t controller_supported_features = AVRCP_FEATURE_MASK_CATEGORY_MONITOR_OR_AMPLIFIER;
    avrcp_controller_create_sdp_record(sdp_avrcp_controller_service_buffer, 0x10003, controller_supported_features, NULL, NULL);
    sdp_register_service(sdp_avrcp_controller_service_buffer);

    // Register Device ID (PnP) service SDP record
    memset(device_id_sdp_service_buffer, 0, sizeof(device_id_sdp_service_buffer));
    device_id_create_sdp_record(device_id_sdp_service_buffer, 0x10004, DEVICE_ID_VENDOR_ID_SOURCE_BLUETOOTH, BLUETOOTH_COMPANY_ID_BLUEKITCHEN_GMBH, 1, 1);
    sdp_register_service(device_id_sdp_service_buffer);

    // Set local name with a template Bluetooth address, that will be automatically
    // replaced with a actual address once it is available, i.e. when BTstack boots
    // up and starts talking to a Bluetooth module.
    gap_set_local_name("A2DP Source 00:00:00:00:00:00");
    gap_discoverable_control(1);
    gap_set_class_of_device(0x200408);
    
    // Register for HCI events.
    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    a2dp_demo_hexcmod_configure_sample_rate(current_sample_rate);
    
    // Parse human readable Bluetooth address.
    sscanf_bd_addr(device_addr_string, device_addr);
    return 0;
}
/* LISTING_END */

static void a2dp_demo_hexcmod_configure_sample_rate(int sample_rate){
    if (!hxcmod_initialized){
        hxcmod_initialized = hxcmod_init(&mod_context);
        if (!hxcmod_initialized) {
            sprintf(out_str, "could not initialize hxcmod\n \r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            return;
        }
    }
    current_sample_rate = sample_rate;
    media_tracker.sbc_storage_count = 0;
    media_tracker.samples_ready = 0;
    hxcmod_unload(&mod_context);
    hxcmod_setcfg(&mod_context, current_sample_rate, 16, 1, 1, 1);
    hxcmod_load(&mod_context, (void *) &mod_data, mod_len);
}

static void a2dp_demo_send_media_packet(void){
    int num_bytes_in_frame = btstack_sbc_encoder_sbc_buffer_length();
    int bytes_in_storage = media_tracker.sbc_storage_count;
    uint8_t num_frames = bytes_in_storage / num_bytes_in_frame;
    // Prepend SBC Header
    media_tracker.sbc_storage[0] = num_frames;  // (fragmentation << 7) | (starting_packet << 6) | (last_packet << 5) | num_frames;
    avdtp_source_stream_send_media_payload_rtp(media_tracker.a2dp_cid, media_tracker.local_seid, 0, media_tracker.sbc_storage, bytes_in_storage + 1);

    media_tracker.sbc_storage_count = 0;
    media_tracker.sbc_ready_to_send = 0;
}

static void produce_sine_audio(int16_t * pcm_buffer, int num_samples_to_write){
    int count;
    for (count = 0; count < num_samples_to_write ; count++){
        switch (current_sample_rate){
            case 44100:
                pcm_buffer[count * 2]     = sine_int16_44100[sine_phase];
                pcm_buffer[count * 2 + 1] = sine_int16_44100[sine_phase];
                sine_phase++;
                if (sine_phase >= num_samples_sine_int16_44100){
                    sine_phase -= num_samples_sine_int16_44100;
                }
                break;
            case 48000:
                pcm_buffer[count * 2]     = sine_int16_48000[sine_phase];
                pcm_buffer[count * 2 + 1] = sine_int16_48000[sine_phase];
                sine_phase++;               
                if (sine_phase >= num_samples_sine_int16_48000){
                    sine_phase -= num_samples_sine_int16_48000;
                }
                break;
            default:
                break;
        }   
    }
}

static void produce_mod_audio(int16_t * pcm_buffer, int num_samples_to_write){
    hxcmod_fillbuffer(&mod_context, (unsigned short *) &pcm_buffer[0], num_samples_to_write, &trkbuf);
}

static int8_t produce_i2s_audio(int16_t * pcm_buffer, int num_samples_to_write){
    
    /* Receive data from byte buffer */
	size_t item_size;

	uint8_t *buffer = (uint8_t *)xRingbufferReceiveUpTo(i2s_ringbuf_handle, &item_size, 0, num_samples_to_write * 4);

    if (item_size == 0 || buffer == NULL)
    {
        int count;
        for (count = 0; count < num_samples_to_write ; count++)
        {
            switch (current_sample_rate)
            {
                case 44100:
                    pcm_buffer[count * 2]     = silence_int16_48000[sine_phase];
                    pcm_buffer[count * 2 + 1] = silence_int16_48000[sine_phase];
                    sine_phase++;
                    if (sine_phase >= num_samples_sine_int16_44100){
                        sine_phase -= num_samples_sine_int16_44100;
                    }
                    break;
                case 48000:
                    pcm_buffer[count * 2]     = silence_int16_48000[sine_phase];
                    pcm_buffer[count * 2 + 1] = silence_int16_48000[sine_phase];
                    sine_phase++;               
                    if (sine_phase >= num_samples_sine_int16_48000){
                        sine_phase -= num_samples_sine_int16_48000;
                    }
                    break;
                default:
                    break;
            }   
        }
        return 0;
    }
    else
    {
        int16_t *buf16 = (int16_t *)buffer;

        /* Copy data over */
        memcpy(pcm_buffer, buf16, item_size);

        /* Remove buffer */
        vRingbufferReturnItem(i2s_ringbuf_handle, (void *)buffer);
        
        return 0;
    }    
}


static void produce_audio(int16_t * pcm_buffer, int num_samples){
    switch (data_source){
        case STREAM_SINE:
            produce_sine_audio(pcm_buffer, num_samples);
            break;
        case STREAM_MOD:
            produce_mod_audio(pcm_buffer, num_samples);
            break;
        case STREAM_I2S:
            produce_i2s_audio(pcm_buffer, num_samples);
            break;
        default:
            break;
    }    
#ifdef VOLUME_REDUCTION
    int i;
    for (i=0;i<num_samples*2;i++){
        if (pcm_buffer[i] > 0){
            pcm_buffer[i] =     pcm_buffer[i]  >> VOLUME_REDUCTION;
        } else {
            pcm_buffer[i] = -((-pcm_buffer[i]) >> VOLUME_REDUCTION);
        }
    }
#endif
}

static int a2dp_demo_fill_sbc_audio_buffer(a2dp_media_sending_context_t * context){
    // perform sbc encoding
    int total_num_bytes_read = 0;
    unsigned int num_audio_samples_per_sbc_buffer = btstack_sbc_encoder_num_audio_frames();
    while (context->samples_ready >= num_audio_samples_per_sbc_buffer
        && (context->max_media_payload_size - context->sbc_storage_count) >= btstack_sbc_encoder_sbc_buffer_length()){

        int16_t pcm_frame[256*NUM_CHANNELS];

        produce_audio(pcm_frame, num_audio_samples_per_sbc_buffer);
        btstack_sbc_encoder_process_data(pcm_frame);
        
        uint16_t sbc_frame_size = btstack_sbc_encoder_sbc_buffer_length(); 
        uint8_t * sbc_frame = btstack_sbc_encoder_sbc_buffer();
        
        total_num_bytes_read += num_audio_samples_per_sbc_buffer;
        // first byte in sbc storage contains sbc media header
        memcpy(&context->sbc_storage[1 + context->sbc_storage_count], sbc_frame, sbc_frame_size);
        context->sbc_storage_count += sbc_frame_size;
        context->samples_ready -= num_audio_samples_per_sbc_buffer;
    }
    return total_num_bytes_read;
}

static void a2dp_demo_audio_timeout_handler(btstack_timer_source_t * timer){
    a2dp_media_sending_context_t * context = (a2dp_media_sending_context_t *) btstack_run_loop_get_timer_context(timer);
    btstack_run_loop_set_timer(&context->audio_timer, AUDIO_TIMEOUT_MS); 
    btstack_run_loop_add_timer(&context->audio_timer);
    uint32_t now = btstack_run_loop_get_time_ms();

    uint32_t update_period_ms = AUDIO_TIMEOUT_MS;
    if (context->time_audio_data_sent > 0){
        update_period_ms = now - context->time_audio_data_sent;
    } 

    uint32_t num_samples = (update_period_ms * current_sample_rate) / 1000;
    context->acc_num_missed_samples += (update_period_ms * current_sample_rate) % 1000;
    
    while (context->acc_num_missed_samples >= 1000){
        num_samples++;
        context->acc_num_missed_samples -= 1000;
    }
    context->time_audio_data_sent = now;
    context->samples_ready += num_samples;

    if (context->sbc_ready_to_send) return;

    a2dp_demo_fill_sbc_audio_buffer(context);

    if ((context->sbc_storage_count + btstack_sbc_encoder_sbc_buffer_length()) > context->max_media_payload_size){
        // schedule sending
        context->sbc_ready_to_send = 1;
        a2dp_source_stream_endpoint_request_can_send_now(context->a2dp_cid, context->local_seid);
    }
}

static void a2dp_demo_timer_start(a2dp_media_sending_context_t * context){
    context->max_media_payload_size = btstack_min(a2dp_max_media_payload_size(context->a2dp_cid, context->local_seid), SBC_STORAGE_SIZE);
    context->sbc_storage_count = 0;
    context->sbc_ready_to_send = 0;
    context->streaming = 1;
    btstack_run_loop_remove_timer(&context->audio_timer);
    btstack_run_loop_set_timer_handler(&context->audio_timer, a2dp_demo_audio_timeout_handler);
    btstack_run_loop_set_timer_context(&context->audio_timer, context);
    btstack_run_loop_set_timer(&context->audio_timer, AUDIO_TIMEOUT_MS); 
    btstack_run_loop_add_timer(&context->audio_timer);
}

static void a2dp_demo_timer_stop(a2dp_media_sending_context_t * context){
    context->time_audio_data_sent = 0;
    context->acc_num_missed_samples = 0;
    context->samples_ready = 0;
    context->streaming = 1;
    context->sbc_storage_count = 0;
    context->sbc_ready_to_send = 0;
    btstack_run_loop_remove_timer(&context->audio_timer);
} 

static void dump_sbc_configuration(media_codec_configuration_sbc_t * configuration){
    sprintf(out_str, "Received media codec configuration:\n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "    - num_channels: %d\n \r", configuration->num_channels);
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "    - sampling_frequency: %d\n \r", configuration->sampling_frequency);
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "    - channel_mode: %d\n \r", configuration->channel_mode);
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "    - block_length: %d\n \r", configuration->block_length);
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "    - subbands: %d\n \r", configuration->subbands);
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "    - allocation_method: %d\n \r", configuration->allocation_method);
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "    - bitpool_value [%d, %d] \n \r", configuration->min_bitpool_value, configuration->max_bitpool_value);
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
}

static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    if (packet_type != HCI_EVENT_PACKET) return;

#ifndef HAVE_BTSTACK_STDIN
    if (hci_event_packet_get_type(packet) == BTSTACK_EVENT_STATE){
        if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
        sprintf(out_str, "Create A2DP Source connection to addr %s.\n \r", bd_addr_to_str(device_addr));
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

        uint8_t status = a2dp_source_establish_stream(device_addr, &media_tracker.a2dp_cid);
        if (status != ERROR_CODE_SUCCESS){
            sprintf(out_str, "Could not perform command, status 0x%2x\n \r", status);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

        }
        return;
    }
#endif

    if (hci_event_packet_get_type(packet) == HCI_EVENT_PIN_CODE_REQUEST) {
        bd_addr_t address;
        sprintf(out_str, "Pin code request - using '0000'\n \r");
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

        hci_event_pin_code_request_get_bd_addr(packet, address);
        gap_pin_code_response(address, "0000");
    }
}

static void a2dp_source_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    uint8_t status;
    uint8_t local_seid;
    bd_addr_t address;
    uint16_t cid;

    avdtp_channel_mode_t channel_mode;
    uint8_t allocation_method;

    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_A2DP_META) return;

    switch (hci_event_a2dp_meta_get_subevent_code(packet)){
        case A2DP_SUBEVENT_SIGNALING_CONNECTION_ESTABLISHED:
            a2dp_subevent_signaling_connection_established_get_bd_addr(packet, address);
            cid = a2dp_subevent_signaling_connection_established_get_a2dp_cid(packet);
            status = a2dp_subevent_signaling_connection_established_get_status(packet);

            if (status != ERROR_CODE_SUCCESS){
                sprintf(out_str, "A2DP Source: Connection failed, status 0x%02x, cid 0x%02x, a2dp_cid 0x%02x \n \r", status, cid, media_tracker.a2dp_cid);
                uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

                media_tracker.a2dp_cid = 0;
                break;
            }
            media_tracker.a2dp_cid = cid;
            media_tracker.volume = 32;

            sprintf(out_str, "A2DP Source: Connected to address %s, a2dp cid 0x%02x, local seid 0x%02x.\n \r", bd_addr_to_str(address), media_tracker.a2dp_cid, media_tracker.local_seid);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            break;

         case A2DP_SUBEVENT_SIGNALING_MEDIA_CODEC_SBC_CONFIGURATION:{
            cid  = avdtp_subevent_signaling_media_codec_sbc_configuration_get_avdtp_cid(packet);
            if (cid != media_tracker.a2dp_cid) return;

            media_tracker.remote_seid = a2dp_subevent_signaling_media_codec_sbc_configuration_get_remote_seid(packet);
            
            sbc_configuration.reconfigure = a2dp_subevent_signaling_media_codec_sbc_configuration_get_reconfigure(packet);
            sbc_configuration.num_channels = a2dp_subevent_signaling_media_codec_sbc_configuration_get_num_channels(packet);
            sbc_configuration.sampling_frequency = a2dp_subevent_signaling_media_codec_sbc_configuration_get_sampling_frequency(packet);
            sbc_configuration.block_length = a2dp_subevent_signaling_media_codec_sbc_configuration_get_block_length(packet);
            sbc_configuration.subbands = a2dp_subevent_signaling_media_codec_sbc_configuration_get_subbands(packet);
            sbc_configuration.min_bitpool_value = a2dp_subevent_signaling_media_codec_sbc_configuration_get_min_bitpool_value(packet);
            sbc_configuration.max_bitpool_value = a2dp_subevent_signaling_media_codec_sbc_configuration_get_max_bitpool_value(packet);
            
            channel_mode = (avdtp_channel_mode_t) a2dp_subevent_signaling_media_codec_sbc_configuration_get_channel_mode(packet);
            allocation_method = a2dp_subevent_signaling_media_codec_sbc_configuration_get_allocation_method(packet);
            
            sprintf(out_str, "A2DP Source: Received SBC codec configuration, sampling frequency %u, a2dp_cid 0x%02x, local seid 0x%02x, remote seid 0x%02x.\n \r", 
                sbc_configuration.sampling_frequency, cid,
                   a2dp_subevent_signaling_media_codec_sbc_configuration_get_local_seid(packet),
                   a2dp_subevent_signaling_media_codec_sbc_configuration_get_remote_seid(packet));
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            
            // Adapt Bluetooth spec definition to SBC Encoder expected input
            sbc_configuration.allocation_method = (btstack_sbc_allocation_method_t)(allocation_method - 1);
            switch (channel_mode){
                case AVDTP_CHANNEL_MODE_JOINT_STEREO:
                    sbc_configuration.channel_mode = SBC_CHANNEL_MODE_JOINT_STEREO;
                    break;
                case AVDTP_CHANNEL_MODE_STEREO:
                    sbc_configuration.channel_mode = SBC_CHANNEL_MODE_STEREO;
                    break;
                case AVDTP_CHANNEL_MODE_DUAL_CHANNEL:
                    sbc_configuration.channel_mode = SBC_CHANNEL_MODE_DUAL_CHANNEL;
                    break;
                case AVDTP_CHANNEL_MODE_MONO:
                    sbc_configuration.channel_mode = SBC_CHANNEL_MODE_MONO;
                    break;
                default:
                    btstack_assert(false);
                    break;
            }
            dump_sbc_configuration(&sbc_configuration);

            btstack_sbc_encoder_init(&sbc_encoder_state, SBC_MODE_STANDARD, 
                sbc_configuration.block_length, sbc_configuration.subbands, 
                sbc_configuration.allocation_method, sbc_configuration.sampling_frequency, 
                sbc_configuration.max_bitpool_value,
                sbc_configuration.channel_mode);
            break;
        }  

        case A2DP_SUBEVENT_SIGNALING_DELAY_REPORTING_CAPABILITY:
            sprintf(out_str, "A2DP Source: remote supports delay report, remote seid %d\n \r", 
                avdtp_subevent_signaling_delay_reporting_capability_get_remote_seid(packet));
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            break;
        case A2DP_SUBEVENT_SIGNALING_CAPABILITIES_DONE:
            sprintf(out_str, "A2DP Source: All capabilities reported, remote seid %d\n \r", 
                avdtp_subevent_signaling_capabilities_done_get_remote_seid(packet));
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            break;

        case A2DP_SUBEVENT_SIGNALING_DELAY_REPORT:
            sprintf(out_str, "A2DP Source: Received delay report of %d.%0d ms, local seid %d\n \r", 
                avdtp_subevent_signaling_delay_report_get_delay_100us(packet)/10, avdtp_subevent_signaling_delay_report_get_delay_100us(packet)%10,
                avdtp_subevent_signaling_delay_report_get_local_seid(packet));
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            break;
       
        case A2DP_SUBEVENT_STREAM_ESTABLISHED:
            a2dp_subevent_stream_established_get_bd_addr(packet, address);
            status = a2dp_subevent_stream_established_get_status(packet);
            if (status != ERROR_CODE_SUCCESS){
                sprintf(out_str, "A2DP Source: Stream failed, status 0x%02x.\n \r", status);
                uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
                break;
            }
            
            local_seid = a2dp_subevent_stream_established_get_local_seid(packet);
            cid = a2dp_subevent_stream_established_get_a2dp_cid(packet);
            
            sprintf(out_str, "A2DP Source: Stream established a2dp_cid 0x%02x, local_seid 0x%02x, remote_seid 0x%02x\n \r", cid, local_seid, a2dp_subevent_stream_established_get_remote_seid(packet));
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            a2dp_demo_hexcmod_configure_sample_rate(current_sample_rate);
            media_tracker.stream_opened = 1;
            data_source = STREAM_MOD;
            status = a2dp_source_start_stream(media_tracker.a2dp_cid, media_tracker.local_seid);
            break;

        case A2DP_SUBEVENT_STREAM_RECONFIGURED:
            status = a2dp_subevent_stream_reconfigured_get_status(packet);
            local_seid = a2dp_subevent_stream_reconfigured_get_local_seid(packet);
            cid = a2dp_subevent_stream_reconfigured_get_a2dp_cid(packet);

            if (status != ERROR_CODE_SUCCESS){
                sprintf(out_str, "A2DP Source: Stream reconfiguration failed with status 0x%02x\n \r", status);
                uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

                break;
            }

            sprintf(out_str, "A2DP Source: Stream reconfigured a2dp_cid 0x%02x, local_seid 0x%02x\n \r", cid, local_seid);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            a2dp_demo_hexcmod_configure_sample_rate(new_sample_rate);
            status = a2dp_source_start_stream(media_tracker.a2dp_cid, media_tracker.local_seid);
            break;

        case A2DP_SUBEVENT_STREAM_STARTED:
            local_seid = a2dp_subevent_stream_started_get_local_seid(packet);
            cid = a2dp_subevent_stream_started_get_a2dp_cid(packet);

            play_info.status = AVRCP_PLAYBACK_STATUS_PLAYING;
            if (media_tracker.avrcp_cid){
                avrcp_target_set_now_playing_info(media_tracker.avrcp_cid, &tracks[data_source], sizeof(tracks)/sizeof(avrcp_track_t));
                avrcp_target_set_playback_status(media_tracker.avrcp_cid, AVRCP_PLAYBACK_STATUS_PLAYING);
            }
            a2dp_demo_timer_start(&media_tracker);
            sprintf(out_str, "A2DP Source: Stream started, a2dp_cid 0x%02x, local_seid 0x%02x\n \r", cid, local_seid);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            break;

        case A2DP_SUBEVENT_STREAMING_CAN_SEND_MEDIA_PACKET_NOW:
            local_seid = a2dp_subevent_streaming_can_send_media_packet_now_get_local_seid(packet);
            cid = a2dp_subevent_signaling_media_codec_sbc_configuration_get_a2dp_cid(packet);
            a2dp_demo_send_media_packet();
            break;        

        case A2DP_SUBEVENT_STREAM_SUSPENDED:
            local_seid = a2dp_subevent_stream_suspended_get_local_seid(packet);
            cid = a2dp_subevent_stream_suspended_get_a2dp_cid(packet);
            
            play_info.status = AVRCP_PLAYBACK_STATUS_PAUSED;
            if (media_tracker.avrcp_cid){
                avrcp_target_set_playback_status(media_tracker.avrcp_cid, AVRCP_PLAYBACK_STATUS_PAUSED);
            }
            sprintf(out_str, "A2DP Source: Stream paused, a2dp_cid 0x%02x, local_seid 0x%02x\n \r", cid, local_seid);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            
            a2dp_demo_timer_stop(&media_tracker);
            break;

        case A2DP_SUBEVENT_STREAM_RELEASED:
            play_info.status = AVRCP_PLAYBACK_STATUS_STOPPED;
            cid = a2dp_subevent_stream_released_get_a2dp_cid(packet);
            local_seid = a2dp_subevent_stream_released_get_local_seid(packet);
            
            sprintf(out_str, "A2DP Source: Stream released, a2dp_cid 0x%02x, local_seid 0x%02x\n \r", cid, local_seid);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            if (cid == media_tracker.a2dp_cid) {
                media_tracker.stream_opened = 0;
                sprintf(out_str, "A2DP Source: Stream released.\n \r");
                uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            }
            if (media_tracker.avrcp_cid){
                avrcp_target_set_now_playing_info(media_tracker.avrcp_cid, NULL, sizeof(tracks)/sizeof(avrcp_track_t));
                avrcp_target_set_playback_status(media_tracker.avrcp_cid, AVRCP_PLAYBACK_STATUS_STOPPED);
            }
            a2dp_demo_timer_stop(&media_tracker);
            break;
        case A2DP_SUBEVENT_SIGNALING_CONNECTION_RELEASED:
            cid = a2dp_subevent_signaling_connection_released_get_a2dp_cid(packet);
            if (cid == media_tracker.a2dp_cid) {
                media_tracker.avrcp_cid = 0;
                media_tracker.a2dp_cid = 0;
                sprintf(out_str, "A2DP Source: Signaling released.\n \r");
                uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            }
            break;
        default:
            break; 
    }
}

static void avrcp_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    bd_addr_t event_addr;
    uint16_t local_cid;
    uint8_t  status = ERROR_CODE_SUCCESS;

    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) return;
    
    switch (packet[2]){
        case AVRCP_SUBEVENT_CONNECTION_ESTABLISHED: 
            local_cid = avrcp_subevent_connection_established_get_avrcp_cid(packet);
            status = avrcp_subevent_connection_established_get_status(packet);
            if (status != ERROR_CODE_SUCCESS){
                sprintf(out_str, "AVRCP: Connection failed, local cid 0x%02x, status 0x%02x\n \r", local_cid, status);
                return;
            }
            media_tracker.avrcp_cid = local_cid;
            avrcp_subevent_connection_established_get_bd_addr(packet, event_addr);

            sprintf(out_str, "AVRCP: Channel to %s successfully opened, avrcp_cid 0x%02x\n \r", bd_addr_to_str(event_addr), media_tracker.avrcp_cid);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            avrcp_target_support_event(media_tracker.avrcp_cid, AVRCP_NOTIFICATION_EVENT_PLAYBACK_STATUS_CHANGED);
            avrcp_target_support_event(media_tracker.avrcp_cid, AVRCP_NOTIFICATION_EVENT_TRACK_CHANGED);
            avrcp_target_support_event(media_tracker.avrcp_cid, AVRCP_NOTIFICATION_EVENT_NOW_PLAYING_CONTENT_CHANGED);
            avrcp_target_set_now_playing_info(media_tracker.avrcp_cid, NULL, sizeof(tracks)/sizeof(avrcp_track_t));
            
            sprintf(out_str, "Enable Volume Change notification\n \r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            avrcp_controller_enable_notification(media_tracker.avrcp_cid, AVRCP_NOTIFICATION_EVENT_VOLUME_CHANGED);
            sprintf(out_str, "Enable Battery Status Change notification\n \r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            avrcp_controller_enable_notification(media_tracker.avrcp_cid, AVRCP_NOTIFICATION_EVENT_BATT_STATUS_CHANGED);
            return;
        
        case AVRCP_SUBEVENT_CONNECTION_RELEASED:
            sprintf(out_str, "AVRCP Target: Disconnected, avrcp_cid 0x%02x\n \r", avrcp_subevent_connection_released_get_avrcp_cid(packet));
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            media_tracker.avrcp_cid = 0;
            return;
        default:
            break;
    }

    if (status != ERROR_CODE_SUCCESS){
        sprintf(out_str, "Responding to event 0x%02x failed with status 0x%02x\n \r", packet[2], status);
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
    }
}

static void avrcp_target_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    uint8_t  status = ERROR_CODE_SUCCESS;

    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) return;

    bool button_pressed;
    char const * button_state;
    avrcp_operation_id_t operation_id;

    switch (packet[2]){
        case AVRCP_SUBEVENT_PLAY_STATUS_QUERY:
            status = avrcp_target_play_status(media_tracker.avrcp_cid, play_info.song_length_ms, play_info.song_position_ms, play_info.status);            
            break;
        // case AVRCP_SUBEVENT_NOW_PLAYING_INFO_QUERY:
        //     status = avrcp_target_now_playing_info(avrcp_cid);
        //     break;
        case AVRCP_SUBEVENT_OPERATION:
            operation_id = avrcp_subevent_operation_get_operation_id(packet);
            button_pressed = avrcp_subevent_operation_get_button_pressed(packet) > 0;
            button_state = button_pressed ? "PRESS" : "RELEASE";

            sprintf(out_str, "AVRCP Target: operation %s (%s)\n \r", avrcp_operation2str(operation_id), button_state);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            if (!button_pressed){
                break;
            }
            switch (operation_id) {
                case AVRCP_OPERATION_ID_PLAY:
                    status = a2dp_source_start_stream(media_tracker.a2dp_cid, media_tracker.local_seid);
                    break;
                case AVRCP_OPERATION_ID_PAUSE:
                    status = a2dp_source_pause_stream(media_tracker.a2dp_cid, media_tracker.local_seid);
                    break;
                case AVRCP_OPERATION_ID_STOP:
                    status = a2dp_source_disconnect(media_tracker.a2dp_cid);
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }

    if (status != ERROR_CODE_SUCCESS){
        sprintf(out_str, "Responding to event 0x%02x failed with status 0x%02x\n \r", packet[2], status);
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
    }
}

static void avrcp_controller_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);
    
    if (packet_type != HCI_EVENT_PACKET) return;
    if (hci_event_packet_get_type(packet) != HCI_EVENT_AVRCP_META) return;
    if (!media_tracker.avrcp_cid) return;
    
    switch (packet[2]){
        case AVRCP_SUBEVENT_NOTIFICATION_VOLUME_CHANGED:
            sprintf(out_str, "AVRCP Controller: Notification Absolute Volume %d %%\n \r", avrcp_subevent_notification_volume_changed_get_absolute_volume(packet) * 100 / 127);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            break;
        case AVRCP_SUBEVENT_NOTIFICATION_EVENT_BATT_STATUS_CHANGED:
            // see avrcp_battery_status_t
            sprintf(out_str, "AVRCP Controller: Notification Battery Status %d\n \r", avrcp_subevent_notification_event_batt_status_changed_get_battery_status(packet));
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            break;
        case AVRCP_SUBEVENT_NOTIFICATION_STATE:
            sprintf(out_str, "AVRCP Controller: Notification %s - %s\n \r", 
                avrcp_event2str(avrcp_subevent_notification_state_get_event_id(packet)), 
                avrcp_subevent_notification_state_get_enabled(packet) != 0 ? "enabled" : "disabled");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            break;
        default:
            break;
    }  
}

#ifdef HAVE_BTSTACK_STDIN

static void show_load_mode(void)
{
    sprintf(out_str, "\n \r     ############## Bluetooth streamer (A2DP Source/AVRCP) ##############\n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "       - Select load mode: \n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "     a      - A2DP Source mode\n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "     s      - Scanner mode\n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "     d      - Stop scan and show avalible targets to connect \n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "     From <1> to <0>  - Choose device for connect  \n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "     r      - Reset device \n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
}

static void show_usage(void)
{
    bd_addr_t iut_address;

    gap_local_bd_addr(iut_address);

    sprintf(out_str, "\n \r     ############## Bluetooth streamer (A2DP Source/AVRCP) (ESP BT MAC: %s) for %s ##############\n \r", bd_addr_to_str(iut_address), bd_addr_to_str(devices[device_id].address));
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "     b      - A2DP Source create connection to addr %s\n \r", bd_addr_to_str(devices[device_id].address));
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "     B      - A2DP Source disconnect\n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "     c      - AVRCP create connection to addr %s\n \r", bd_addr_to_str(devices[device_id].address));
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "     C      - AVRCP disconnect\n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "     D      - delete all link keys\n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "     x      - start streaming sine\n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "     i      - start streaming from external I2S source\n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
    
    if (hxcmod_initialized){
        sprintf(out_str, "     z      - start streaming '%s'\n \r", mod_name);
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
    }

    sprintf(out_str, "     p      - pause streaming\n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "     w      - reconfigure stream for 44100 Hz\n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "     e      - reconfigure stream for 48000 Hz\n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "     t      - volume up\n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "     T      - volume down\n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "     v      - volume up (via set absolute volume)\n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "     V      - volume down (via set absolute volume)\n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "     r      - reset ESP \n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

    sprintf(out_str, "---\n \r");
    uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
}

static void stdin_process_bootstage(char cmd){
    
     uint8_t status = ERROR_CODE_SUCCESS;
     int err;

      switch (cmd){
        case 'a':

            #ifdef HAVE_BTSTACK_STDIN
                btstack_stdin_setup(stdin_process);
            #endif

            sprintf(out_str, "\n\rStarted A2DP streamer mode. Press <Space> to show avaliable commands \n\r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            int err = a2dp_source_and_avrcp_services_init();
            if (err) sprintf(out_str, "\n\r\n\rError init a2dp source and avrcp services. Code %d \n\r", err);;

            // turn on!
            hci_power_control(HCI_POWER_ON);

            break;

        case 's':

            sprintf(out_str, "\n\rStarted scanner mode. Wait for results... \n\r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            // enabled EIR
            hci_set_inquiry_mode(INQUIRY_MODE_RSSI_AND_EIR);

            hci_event_callback_registration.callback = &packet_handler;
            hci_add_event_handler(&hci_event_callback_registration);

            // turn on!
            hci_power_control(HCI_POWER_ON);           

            break;

        case 'd':

            gap_inquiry_stop();

            hci_event_callback_registration.callback = &hci_packet_handler;
            hci_add_event_handler(&hci_event_callback_registration);

            sprintf(out_str, "\n\rPrint avaliable targets to connect: \n\r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            int i;
            for (i = 0; i < deviceCount; i++) 
            {                
                sprintf(out_str, " - %d. / Name: %s / RSSI: %ddBm / MAC: %s \n \r", i + 1, devices[i].name, devices[i].rssi, bd_addr_to_str(devices[i].address));
                uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str)); 
            }
            break;

        case 'r':

            sprintf(out_str, "\n\rCommand restart received! \n\r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            sprintf(out_str, "\n\r\n\rPlease, wait...\n\r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            esp_restart();

            break;

        case '1': device_id = 0;

            sprintf(out_str, "\n\rChoosen device for connect with MAC \"%s\" with name \"%s\" and RSSI %d \n \r", bd_addr_to_str(devices[device_id].address), devices[device_id].name, devices[device_id].rssi);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            device_addr_string = bd_addr_to_str(devices[device_id].address);

            #ifdef HAVE_BTSTACK_STDIN
                btstack_stdin_setup(stdin_process);
            #endif

            sprintf(out_str, "\n\rStarted A2DP streamer mode. Press <Space> to show avaliable commands \n\r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            err = a2dp_source_and_avrcp_services_init();
            if (err) sprintf(out_str, "\n\r\n\rError init a2dp source and avrcp services. Code %d \n\r", err);;

            // turn on!
            hci_power_control(HCI_POWER_ON);
           
            break;  

        case '2': device_id = 1; 

            sprintf(out_str, "\n\rChoosen device for connect with MAC \"%s\" with name \"%s\" and RSSI %d \n \r", bd_addr_to_str(devices[device_id].address), devices[device_id].name, devices[device_id].rssi);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            device_addr_string = bd_addr_to_str(devices[device_id].address);

            #ifdef HAVE_BTSTACK_STDIN
                btstack_stdin_setup(stdin_process);
            #endif

            sprintf(out_str, "\n\rStarted A2DP streamer mode. Press <Space> to show avaliable commands \n\r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            err = a2dp_source_and_avrcp_services_init();
            if (err) sprintf(out_str, "\n\r\n\rError init a2dp source and avrcp services. Code %d \n\r", err);;

            // turn on!
            hci_power_control(HCI_POWER_ON);
           
            break;             

        case '3': device_id = 2; 

            sprintf(out_str, "\n\rChoosen device for connect with MAC \"%s\" with name \"%s\" and RSSI %d \n \r", bd_addr_to_str(devices[device_id].address), devices[device_id].name, devices[device_id].rssi);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            device_addr_string = bd_addr_to_str(devices[device_id].address);

            #ifdef HAVE_BTSTACK_STDIN
                btstack_stdin_setup(stdin_process);
            #endif

            sprintf(out_str, "\n\rStarted A2DP streamer mode. Press <Space> to show avaliable commands \n\r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            err = a2dp_source_and_avrcp_services_init();
            if (err) sprintf(out_str, "\n\r\n\rError init a2dp source and avrcp services. Code %d \n\r", err);;

            // turn on!
            hci_power_control(HCI_POWER_ON);
           
            break;  

        case '4': device_id = 3; 

            sprintf(out_str, "\n\rChoosen device for connect with MAC \"%s\" with name \"%s\" and RSSI %d \n \r", bd_addr_to_str(devices[device_id].address), devices[device_id].name, devices[device_id].rssi);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            device_addr_string = bd_addr_to_str(devices[device_id].address);

            #ifdef HAVE_BTSTACK_STDIN
                btstack_stdin_setup(stdin_process);
            #endif

            sprintf(out_str, "\n\rStarted A2DP streamer mode. Press <Space> to show avaliable commands \n\r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            err = a2dp_source_and_avrcp_services_init();
            if (err) sprintf(out_str, "\n\r\n\rError init a2dp source and avrcp services. Code %d \n\r", err);;

            // turn on!
            hci_power_control(HCI_POWER_ON);
           
            break;  

        case '5': device_id = 4;    

            sprintf(out_str, "\n\rChoosen device for connect with MAC \"%s\" with name \"%s\" and RSSI %d \n \r", bd_addr_to_str(devices[device_id].address), devices[device_id].name, devices[device_id].rssi);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            device_addr_string = bd_addr_to_str(devices[device_id].address);

            #ifdef HAVE_BTSTACK_STDIN
                btstack_stdin_setup(stdin_process);
            #endif

            sprintf(out_str, "\n\rStarted A2DP streamer mode. Press <Space> to show avaliable commands \n\r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            err = a2dp_source_and_avrcp_services_init();
            if (err) sprintf(out_str, "\n\r\n\rError init a2dp source and avrcp services. Code %d \n\r", err);;

            // turn on!
            hci_power_control(HCI_POWER_ON);
           
            break;           

        case '6': device_id = 5;  

            sprintf(out_str, "\n\rChoosen device for connect with MAC \"%s\" with name \"%s\" and RSSI %d \n \r", bd_addr_to_str(devices[device_id].address), devices[device_id].name, devices[device_id].rssi);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            device_addr_string = bd_addr_to_str(devices[device_id].address);

            #ifdef HAVE_BTSTACK_STDIN
                btstack_stdin_setup(stdin_process);
            #endif

            sprintf(out_str, "\n\rStarted A2DP streamer mode. Press <Space> to show avaliable commands \n\r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            err = a2dp_source_and_avrcp_services_init();
            if (err) sprintf(out_str, "\n\r\n\rError init a2dp source and avrcp services. Code %d \n\r", err);;

            // turn on!
            hci_power_control(HCI_POWER_ON);
           
            break;  
          
        case '7': device_id = 6;       

            sprintf(out_str, "\n\rChoosen device for connect with MAC \"%s\" with name \"%s\" and RSSI %d \n \r", bd_addr_to_str(devices[device_id].address), devices[device_id].name, devices[device_id].rssi);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            device_addr_string = bd_addr_to_str(devices[device_id].address);

            #ifdef HAVE_BTSTACK_STDIN
                btstack_stdin_setup(stdin_process);
            #endif

            sprintf(out_str, "\n\rStarted A2DP streamer mode. Press <Space> to show avaliable commands \n\r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            err = a2dp_source_and_avrcp_services_init();
            if (err) sprintf(out_str, "\n\r\n\rError init a2dp source and avrcp services. Code %d \n\r", err);;

            // turn on!
            hci_power_control(HCI_POWER_ON);
           
            break;       
        
        case '8': device_id = 7; 

            sprintf(out_str, "\n\rChoosen device for connect with MAC \"%s\" with name \"%s\" and RSSI %d \n \r", bd_addr_to_str(devices[device_id].address), devices[device_id].name, devices[device_id].rssi);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            device_addr_string = bd_addr_to_str(devices[device_id].address);

            #ifdef HAVE_BTSTACK_STDIN
                btstack_stdin_setup(stdin_process);
            #endif

            sprintf(out_str, "\n\rStarted A2DP streamer mode. Press <Space> to show avaliable commands \n\r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            err = a2dp_source_and_avrcp_services_init();
            if (err) sprintf(out_str, "\n\r\n\rError init a2dp source and avrcp services. Code %d \n\r", err);;

            // turn on!
            hci_power_control(HCI_POWER_ON);
           
            break;   

        case '9': device_id = 8;    

            sprintf(out_str, "\n\rChoosen device for connect with MAC \"%s\" with name \"%s\" and RSSI %d \n \r", bd_addr_to_str(devices[device_id].address), devices[device_id].name, devices[device_id].rssi);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            device_addr_string = bd_addr_to_str(devices[device_id].address);

            #ifdef HAVE_BTSTACK_STDIN
                btstack_stdin_setup(stdin_process);
            #endif

            sprintf(out_str, "\n\rStarted A2DP streamer mode. Press <Space> to show avaliable commands \n\r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            err = a2dp_source_and_avrcp_services_init();
            if (err) sprintf(out_str, "\n\r\n\rError init a2dp source and avrcp services. Code %d \n\r", err);;

            // turn on!
            hci_power_control(HCI_POWER_ON);
           
            break;         

        case '0': device_id = 9; 
            
            sprintf(out_str, "\n\rChoosen device for connect with MAC \"%s\" with name \"%s\" and RSSI %d \n \r", bd_addr_to_str(devices[device_id].address), devices[device_id].name, devices[device_id].rssi);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            device_addr_string = bd_addr_to_str(devices[device_id].address);

            #ifdef HAVE_BTSTACK_STDIN
                btstack_stdin_setup(stdin_process);
            #endif

            sprintf(out_str, "\n\rStarted A2DP streamer mode. Press <Space> to show avaliable commands \n\r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            err = a2dp_source_and_avrcp_services_init();
            if (err) sprintf(out_str, "\n\r\n\rError init a2dp source and avrcp services. Code %d \n\r", err);;

            // turn on!
            hci_power_control(HCI_POWER_ON);
           
            break;       

        default:
            
            return;
    }
    if (status != ERROR_CODE_SUCCESS){
        sprintf(out_str, "Could not perform command \'%c\', status 0x%02x\n \r", cmd, status);
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
    }
}


static void stdin_process(char cmd){
    uint8_t status = ERROR_CODE_SUCCESS;
    switch (cmd){
        case 'b':
            status = a2dp_source_establish_stream(device_addr, &media_tracker.a2dp_cid);
            sprintf(out_str, "%c - Create A2DP Source connection to addr %s, cid 0x%02x.\n \r", cmd, bd_addr_to_str(device_addr), media_tracker.a2dp_cid);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            break;
        case 'B':
            sprintf(out_str, "%c - A2DP Source Disconnect from cid 0x%2x\n \r", cmd, media_tracker.a2dp_cid);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            status = a2dp_source_disconnect(media_tracker.a2dp_cid);
            break;
        case 'c':
            sprintf(out_str, "%c - Create AVRCP connection to addr %s.\n \r", cmd, bd_addr_to_str(device_addr));
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            status = avrcp_connect(device_addr, &media_tracker.avrcp_cid);
            break;
        case 'C':
            sprintf(out_str, "%c - AVRCP disconnect\n \r", cmd);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            status = avrcp_disconnect(media_tracker.avrcp_cid);
            break;
        case 'D':
            sprintf(out_str, "Deleting all link keys\n \r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            gap_delete_all_link_keys();
            break;
        case '\n':
        case '\r':
            show_usage();
            break;

        case 't':
            sprintf(out_str, " - volume up\n \r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            status = avrcp_controller_volume_up(media_tracker.avrcp_cid);
            break;
        case 'T':
            sprintf(out_str, " - volume down\n \r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            status = avrcp_controller_volume_down(media_tracker.avrcp_cid);
            break;

        case 'v':
            if (media_tracker.volume > 117){
                media_tracker.volume = 127;
            } else {
                media_tracker.volume += 10;
            }
            sprintf(out_str, " - volume up (via set absolute volume) %d%% (%d)\n \r",  media_tracker.volume * 100 / 127,  media_tracker.volume);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            status = avrcp_controller_set_absolute_volume(media_tracker.avrcp_cid, media_tracker.volume);
            break;
        case 'V':
            if (media_tracker.volume < 10){
                media_tracker.volume = 0;
            } else {
                media_tracker.volume -= 10;
            }
            sprintf(out_str, " - volume down (via set absolute volume) %d%% (%d)\n \r",  media_tracker.volume * 100 / 127,  media_tracker.volume);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            status = avrcp_controller_set_absolute_volume(media_tracker.avrcp_cid, media_tracker.volume);
            break;
        
        case 'x':
            if (media_tracker.avrcp_cid){
                avrcp_target_set_now_playing_info(media_tracker.avrcp_cid, &tracks[data_source], sizeof(tracks)/sizeof(avrcp_track_t));
            }
            sprintf(out_str, "%c - Play sine.\n \r", cmd);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            data_source = STREAM_SINE;
            if (!media_tracker.stream_opened) break;
            status = a2dp_source_start_stream(media_tracker.a2dp_cid, media_tracker.local_seid);
            break;


        case 'z':
            if (media_tracker.avrcp_cid){
                avrcp_target_set_now_playing_info(media_tracker.avrcp_cid, &tracks[data_source], sizeof(tracks)/sizeof(avrcp_track_t));
            }
            sprintf(out_str, "%c - Play mod.\n \r", cmd);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            data_source = STREAM_MOD;
            if (!media_tracker.stream_opened) break;
            status = a2dp_source_start_stream(media_tracker.a2dp_cid, media_tracker.local_seid);
            break;
        
        case 'i':
            if (media_tracker.avrcp_cid){
                avrcp_target_set_now_playing_info(media_tracker.avrcp_cid, &tracks[data_source], sizeof(tracks)/sizeof(avrcp_track_t));
            }
            sprintf(out_str, "%c - Play mod.\n \r", cmd);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            data_source = STREAM_I2S;
            if (!media_tracker.stream_opened) break;
            status = a2dp_source_start_stream(media_tracker.a2dp_cid, media_tracker.local_seid);
            break;

        case 'p':
            if (!media_tracker.stream_opened) break;
            sprintf(out_str, "%c - Pause stream.\n \r", cmd);
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            status = a2dp_source_pause_stream(media_tracker.a2dp_cid, media_tracker.local_seid);
            break;
        
        case 'w':
            if (!media_tracker.stream_opened) break;
            if (play_info.status == AVRCP_PLAYBACK_STATUS_PLAYING){
                sprintf(out_str, "Stream cannot be reconfigured while playing, please pause stream first\n \r");
                uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
                break;
            }
            new_sample_rate = 44100;
            if (current_sample_rate == new_sample_rate){
                sprintf(out_str, "%c - Stream already configured for %d Hz.\n \r", cmd, new_sample_rate);
                uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            } else {
                sprintf(out_str, "%c - Reconfigure for %d Hz.\n \r", cmd, new_sample_rate);
                uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
                status = a2dp_source_reconfigure_stream_sampling_frequency(media_tracker.a2dp_cid, new_sample_rate);
            }
            break;

        case 'e':
            if (!media_tracker.stream_opened) break;
            if (play_info.status == AVRCP_PLAYBACK_STATUS_PLAYING){
                sprintf(out_str, "Stream cannot be reconfigured while playing, please pause stream first\n \r");
                uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
                break;
            }
            new_sample_rate = 48000;
            if (current_sample_rate == new_sample_rate){
                sprintf(out_str, "%c - Stream already configured for %d Hz.\n \r", cmd, new_sample_rate);
                uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
            } else {
                sprintf(out_str, "%c - Reconfigure for %d Hz.\n \r", cmd, new_sample_rate);
                uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
                status = a2dp_source_reconfigure_stream_sampling_frequency(media_tracker.a2dp_cid, new_sample_rate);
            }
            break;

        case 'r':

            sprintf(out_str, "\n\rCommand restart received! \n\r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            sprintf(out_str, "\n\r\n\rPlease, wait...\n\r");
            uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));

            esp_restart();

            break;

        default:
            show_usage();
            return;
    }
    if (status != ERROR_CODE_SUCCESS){
        sprintf(out_str, "Could not perform command \'%c\', status 0x%02x\n \r", cmd, status);
        uart_write_bytes(ECHO_UART_PORT_NUM, (const char*)out_str, strlen(out_str));
    }
}
#endif


int btstack_main(int argc, const char * argv[]);
int btstack_main(int argc, const char * argv[]){
    (void)argc;
    (void)argv;

    init_gpio_led();

    #ifdef HAVE_BTSTACK_STDIN
        btstack_stdin_setup(stdin_process_bootstage);
    #endif

    show_load_mode();

    // Enter run loop (forever)
    btstack_run_loop_execute();

    
    return 0;
}
