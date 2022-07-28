# ESP32-A2DP-Source-with-btstack

ESP32 A2DP Source example based on ESP-IDF v.5.0 and BTStack by Bluekitchen.

Supported two working mode:
- Inquiry scanner;
- A2DP source;

Supported three sources for A2DP:
- embedded sine wave;
- embedded 8-bit song that played by tiny [HX Mod Player](http://hxc2001.free.fr)
- I2S external source (48kHz an 16 bit stereo)

After start you can see menu at UART console:
![image](https://user-images.githubusercontent.com/23205055/181525914-041fd3aa-55a5-4df7-9fc0-bc4381201f65.png)

You can scan air for your device and choose it for connection if device found it:
![image](https://user-images.githubusercontent.com/23205055/181526096-52ec1233-b307-4222-aab4-c33c01254278.png)

Avaliable commands after choosing A2DP Sink device:
![image](https://user-images.githubusercontent.com/23205055/181561497-cc1a29a4-e9ef-4050-9da6-7c312f583579.png)
