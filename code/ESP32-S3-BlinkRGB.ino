/* ESP32-S3-BlinkRGB.ino
   ESP32-S3-WROOM (CAM Module) | ESP32-S3-CAM
   ESP32-S3-WROOM-1 Dev N8R2   | ESP32-S3-DevKitC-1 | YD-ESP32-S3

   Demonstrate usage of onboard the RGB LED on some ESP Dev boards.
   Calls to digitalWrite(RGB_BUILTIN, HIGH) -will use the hidden
   RGB driver.   --- But, DO NOT re- #define RGB_BUILTIN, If so,
   digitalWrite() will not work.

   neopixelWrite demonstrates controll of each color channel:
   neopixelWrite(uint8_t Pin, uint8_t R, uint8_t G, uint8_t B)

   NOTE: The ColorOrder is “RGB”, NOT BGR as in some ESP32s'.

   SERIAL MONITOR:         [Both NL & CR  ⤋] [15200 baud  ⤋]
*/

/*******************************************************************
Arduino IDE > Tools                                  [CONFIGURATION]
                       Board: "ESP32S3 Dev Module"
             USB CDC On Boot: "Disabled" * Sketch locks up if ENABLED *
               CPU Frequency: "240MHz (WiFi)"
             USB DFU On Boot: "Disabled"
                  Flash Mode: "QIO 80MHz"
                  Flash Size: "16MB 128Mb"  -or-
                  Flash Size: "8MB 64Mb"   check your board!
    USB Firmware MSC On Boot: "Disabled"
            Partition Scheme: "16M Flash (2MB APP/12.5MB FATFS)"  -or-
            Partition Scheme: "8MB with spiffs (3MB APP/1.5MB SPIFFS)"
                       PSRAM: "QSPI PSRAM"
                 Upload Mode: "UART0/Hardware CDC"
                Upload Speed: "115200"
                    USB Mode: "Hardware CDC and JTAG"
*******************************************************************/

//#define RGB_BUILTIN 48    // DO NOT REDEFINE RGB_BUILTIN
                            // If so, digitalWrite() will not work.

#ifdef  RGB_BRIGHTNESS      // if RGB_BRIGHTNESS is defined...
#undef  RGB_BRIGHTNESS      // un-define it... then
#define RGB_BRIGHTNESS 10   // Change color brightness (max 255)
#endf

void setup() {
  Serial.begin(115200);             // initialize Serial Monitor
  while(!Serial);                   // wait for Serial Port to open

  // No need to initialize the RGB LED!
  rgbLedWrite(RGB_BUILTIN, 0,0,0);  // Make sure RGB NeoPixel is Off
}

void loop() {
  // Blink the RGB LED (White),
  // use the hidden RGB driver built into digitalWrite()
  Serial.println("Blink White!");
  digitalWrite(RGB_BUILTIN, HIGH);  // Turn On the RGB LED (White)
  delay(1000);
  digitalWrite(RGB_BUILTIN, LOW);   // Turn Off the RGB LED
  delay(1000);

  // Now let's Blink the RGB LED Neopixel in COLOR!
  Serial.print("Blink Red!... ");
  rgbLedWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0,0);  // Red
  delay(1000);
  Serial.print("Blink Green!... ");
  rgbLedWrite(RGB_BUILTIN,0,RGB_BRIGHTNESS,0);  // Green
  delay(1000);
  Serial.println("Blink Blue!\n");
  rgbLedWrite(RGB_BUILTIN,0,0,RGB_BRIGHTNESS);  // Blue
  delay(1000);
  rgbLedWrite(RGB_BUILTIN,0,0,0);               // OFF
  delay(1000);
}

/*******************************************************************
Sketch uses 323144 bytes (15%) of program storage space.
  Maximum is 2097152 bytes.
Global variables use 20052 bytes (6%) of dynamic memory,
  leaving 307628 bytes for local variables. Maximum is 327680 bytes.
esptool.py v4.8.1
Serial port COM6
Connecting.....
Chip is ESP32-S3 (QFN56) (revision v0.2)
Features: WiFi, BLE, Embedded PSRAM 8MB (AP_3v3)
Crystal is 40MHz
MAC: cc:ba:97:05:10:8c

Uploading stub...
Running stub...
Stub running...
Configuring flash size...
Erasing flash (this may take a while)...
Chip erase completed successfully in 3.5 seconds.
Compressed 20160 bytes to 12987...
Writing at 0x00000000... (100 %)
Wrote 20160 bytes (12987 compressed) at 0x00000000 in 1.3 seconds
  (effective 127.8 kbit/s)...
Hash of data verified.
Compressed 3072 bytes to 143...
Writing at 0x00008000... (100 %)
Wrote 3072 bytes (143 compressed) at 0x00008000 in 0.0 seconds
  (effective 834.8 kbit/s)...
Hash of data verified.
Compressed 8192 bytes to 47...
Writing at 0x0000e000... (100 %)
Wrote 8192 bytes (47 compressed) at 0x0000e000 in 0.0 seconds
  (effective 2149.3 kbit/s)...
Hash of data verified.
Compressed 323248 bytes to 173462...
Writing at 0x00010000... (9 %)
Writing at 0x0001c196... (18 %)
Writing at 0x000289b0... (27 %)
Writing at 0x0002e12e... (36 %)
Writing at 0x00033c74... (45 %)
Writing at 0x00039417... (54 %)
Writing at 0x0003ec21... (63 %)
Writing at 0x000448ab... (72 %)
Writing at 0x0004a18e... (81 %)
Writing at 0x000553ae... (90 %)
Writing at 0x0005b46a... (100 %)
Wrote 323248 bytes (173462 compressed) at 0x00010000 in 15.4 seconds
  (effective 168.2 kbit/s)...
Hash of data verified.

Leaving...
Hard resetting via RTS pin...
*******************************************************************/

/* -----------------------------------------------------------------
Serial Monitor:

05:05:22.426 -> ESP-ROM:esp32s3-20210327
05:05:22.426 -> Build:Mar 27 2021
05:05:22.426 -> rst:0x1 (POWERON),boot:0x8 (SPI_FAST_FLASH_BOOT)
05:05:22.426 -> SPIWP:0xee
05:05:22.426 -> mode:DIO, clock div:1
05:05:22.426 -> load:0x3fce2820,len:0x118c
05:05:22.426 -> load:0x403c8700,len:0x4
05:05:22.426 -> load:0x403c8704,len:0xc20
05:05:22.426 -> load:0x403cb700,len:0x30e0
05:05:22.426 -> entry 0x403c88b8

05:05:24.663 -> Blink White!
05:05:26.641 -> Blink Red!... Blink Green!... Blink Blue!
05:05:28.659 ->
05:05:30.671 -> Blink White!
05:05:32.641 -> Blink Red!... Blink Green!... Blink Blue!
05:05:34.671 ->
05:05:36.695 -> Blink White!
05:05:38.659 -> Blink Red!... Blink Green!... Blink Blue!
05:05:40.641 -> ...
----------------------------------------------------------------- */
