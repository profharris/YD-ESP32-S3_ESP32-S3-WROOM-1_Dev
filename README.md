# YD-ESP32-S3 ESP32-S3-WROOM-1 Dev N16R8/N8R2 (44-Pin)

***WORK IN PROGRESS*** &nbsp; &nbsp; Prof. Michael P. Harris &nbsp; &nbsp; *05/17/2025*

{ESP32-S3-DevKitC-1 clone}&nbsp;  (11-pins wide × 22-pins)&nbsp; (1.1" × 2.5")

> **VCC-GND Studio**,&nbsp; ESP32-S3 Core Development Board:&nbsp; ‘**YD-ESP32-S3**’.<br/>
> Also availiable as:&nbsp; Binghe ‘**ESP32-S3 Core Development board**’.<br/>
> Also availiable as:&nbsp; ‘**ESP32-S3-WROOM-1 Dev N16R8/N8R2**’.

This dual USB-C,&nbsp; ESP32-S3 Dev Board,&nbsp; is easily recognized by its’ four
in-a-row LEDs...<br/>a WS2812 **RGB LED** (GPIO48), followed by a (Red)
**Power LED**, a (Green) **TX LED**, and a (Blue) **RX LED**.

This _44-pin_ inexpensive workhorse&nbsp; ESP32-S3 Dev Board is also easily
identified by the two unique solder jumper pads on the front,&nbsp; and one on the
backside.&nbsp; Right next to the RGB LED on the right side are a pair of
solder jumber pads labled ‘**RGB**’.&nbsp; On the left side a little lower
than the RGB LED is a second pair of solder jumper pads labled
‘**IN-OUT**’.&nbsp; On the backside of the board, is a third set of solder
jumber pads labled ‘**USB-OTG**’.&nbsp; The function of these solder jumpers
are discussed a little later below under ‘NOTES¹’.

```
/*******************************************************************************
ESP32-S3-WROOM-1 Dev (YD-ESP32-S3)  (44-Pin)     ESP32-S3 Dev Module

    Same pinout as Espressif ESP32-S3-DevKitC-1, except 1-pin space
    wider. Almost (with 1 extra pin top & bottom) the same pinout as
    the ‘ESP32-S3-WROOM’ (CAM Module), and ‘GOOUUU ESP32-S3-CAM’.

Xtensa® 32-bit                               ESP32-S3-WROOM-1 N8R2  ———————————
dual-core LX7            YD-ESP32-S3                                 I²C QWIIC
240MHz, 512KB SRAM     _______________       NO CAMERA MODULE       ———————————
8MB ƒlash, 2MB PSRAM  |  ___   _   __¯|      NO SD-CARD             1   *   GND
WiFi 802.11 b/g/n     | | | |_| |_|   |      GPIO_ = Strapping Pin  2   *   3V3
BLE®5              .——| ' '           |——.                          3 GPIO8 SDA
             3V3  1|o:|ESP32S3-WROOM-1|:o|44 GND                    4 GPIO9 SCL
             3V3  2|o:|               |:¤|43 GPIO43 TX›[U0TXD  ]      ——ALT——
[RESET  ] EN/RST  3|o:| .··. . F© Œ Æ |:¤|42 GPIO44 RX‹[U0RXD  ]    3 GPIO1 SDA
[A3   T4] GPIO4   4|o:| WiFi ß   ____ |:o|41 GPIO1  SDA[A0   T1]    4 GPIO2 SCL
[A4   T5] GPIO5   5|o:|         |QRCD||:o|40 GPIO2  SCL[A1   T2]
[A5   T6] GPIO6   6|o:| °  N8R2 |____||:o|39 GPIO42    [   MTMS]
[A6   T7] GPIO7   7|o:'———————————————':o|38 GPIO41    [   MTDI]     Hardware
[A14    ] GPIO15  8|o:                 :o|37 GPIO40    [   MTDO]     SPI2 HSPI
[A15    ] GPIO16  9|o  :: ‡‡‡    · RST  o|36 GPIO39    [   MTCK]    ———————————
[A16    ] GPIO17 10|o  ¨¨|¯¯¯¬   : [Ø]  o|35 GPIO38    [       ]    GPIO8  RST
[A17    ] GPIO18 11|o  ¨¨|LDO[]  : BOOT •|34 GPIO37    [PSRAM •]    GPIO9  DC
[A7  SDA] GPIO8  12|o  ¨¨|___- ¬¬  [Ø]  •|33 GPIO36    [PSRAM •]    GPIO10 CS
[A2   T3] GPIO3_ 13|o  ·  ‡‡‡  ¨¨       •|32 GPIO35    [PSRAM •]    GPIO11 MOSI
[IN ONLY] GPIO46_14|o      RGB   P T R  o|31 GPIO0_    [BOOT   ]    GPIO12 SCLK
[A8  SCL] GPIO9  15|o  RGB CTRL  R X X  o|30 GPIO45_   [   VSPI]    GPIO13 MISO
[A9  T10] GPIO10 16|o  [¤]  ¥    ¤ ¤ ¤  ¤|29 GPIO48 RGB[WS2812¤]
[A10 T11] GPIO11 17|o           ··· ___ o|28 GPIO47    [       ]
[A11 T12] GPIO12 18|oIN-OUT ‡‡‡ :::|343|o|27 GPIO21    [       ]     Software
[A12 T13] GPIO13 19|o ¥            |___|ø|26 GPIO20    [A19  D+]     SPI —ALT—
[A13 T14] GPIO14 20|o  _____ O T _____  ø|25 GPIO19    [A18  D-]    ———————————
[ IN-OUT]    5V0 21|o | USB |T T| USB | o|24 GND                       -1  MISO
             GND 22|o |  C  |G L|  C  | o|23 GND                    GPIO42 SCLK
                   '——'ESP32'———'UART0'——'                          GPIO41 MOSI
                                             GPIO48 RGB_BUILTIN,    GPIO0  RST
Red PWR LED, Green TX LED, Blue RX LED              LED_BUILTIN     GPIO45 DC
                                                                    GPIO47 CS
ESP32-S3 Pins: 0…18 GPIO, 19…20 D+/D-, 21 GPIO, 22…25 Do Not Exist, GPIO21 BL*
26…32 QSPI ƒlash, 33…34 N/A, 35…42 GPIO, 43…44 TX/RX, 45…48 GPIO.
 pins_arduino.h ~ ESP32-S3-DevKitC

*******************************************************************************/
```
———————————————————————————————————————————————————
# NOTES¹:&nbsp; ‘RGB’ ‘IN-OUT’ ‘USB-OTG’ solder jumper pads

This ‘ESP32 S3 DevKitC1 Clone’ board has a solder pad jumper labled
‘**IN-OUT**’, another ‘**RGB**’ ( _RGB CTRL_ ), and third ‘**USB-OTG**’
on the back;&nbsp; ***--All Open*** on most boards,&nbsp; but I have
one board that came with a SMT capacitor across ‘**RGB**’.&nbsp; It may
be necessary to solder closed some of the jumpers for the devices to
work with your configuration/application.

The ‘**IN-OUT**’ jumper, when closed,&nbsp; bypasses 1 diode,&nbsp; making
USB VBus power coming to 5Vin.&nbsp; If 5Vin is also connected to external
source,&nbsp; it can get back-fed by USB 5V0,&nbsp; which is usually
undesirable.&nbsp; But this USB bus is protected by another diode,&nbsp;
it cannot get back-fed by an external source.

When ‘**IN-OUT**’ is open,&nbsp; 5Vin and USB VBus are separated by a diode,
USB power does not come to 5Vin.

The ‘**USB-OTG**’ jumper on the backside,&nbsp; when closed,&nbsp; connects
together the USB VBus lines from both USB-C connectors.

***REF:*** Third-party ESP32-S3 development boards ‘IN-OUT’ and ‘USB-OTG’
pads - What do they do?
https://www.reddit.com/r/esp32/comments/10rdngp/thirdparty_esp32s3_development_boards_inout_and/?rdt=39953

———————————————————————————————————————————————————
# NOTES²:&nbsp; WS2812 RGB LED (GPIO48)

The RGB LED, when accessed as **LED_BUILTIN**,&nbsp; works with the Arduino IDE
common _digitalWrite()_ command.<br/>
The RGB LED when accessed as **RGB_BUILTIN** works with the _neopixelWrite()_ command.

Arduino IDE:&nbsp; There is a _BlinkRGB.ino_ under the&nbsp; ESP32->GPIO examples
that uses the onboard RGB LED.

***REF:*** https://forum.arduino.cc/t/esp32-s3-devkit-problems/1136923/4

YOU MAY NEED TO ADD:&nbsp;&nbsp; '#define RGB_BUILTIN 48'

———————————————————————————————————————————————————
## YD-ESP32-S3 N16R8 &nbsp;--by VCC-GND Studio

The **YD-ESP32-S3** Core Board is designed by VCC-GND Studio. Visit
www.vcc-gnd.com for purchase.<br/>The device uses the **ESP32-S3-WROOM-1**
SoC chip, which can be used for Internet-of-Things applications and
can also be used for practical applications.

It is equipped with two USB Type-C ports.&nbsp; one is a hardware USB-to-Serial port
via the CH343P (WCH Qinheng),&nbsp; and the other is ESP32-S3 direct USB port.

The YD-ESP32-S3 is an entry-level development board equipped with a 2.4GHz
WiFi 802.11 b/g/n + Bluetooth® LE module **ESP32-S3-WROOM-1**.

Most of the pins of the modules on the board have been lead out to
the pin rows on both sides of the development board.&nbsp; Developers can
easily connect various peripheral devices through jumpers,&nbsp; or plug
the development board into a breadboard for use.

***REF:*** YD-ESP32-S3 N16R8 &nbsp;(ESP32-S3-WROOM-1 Dev N8R2)

———————————————————————————————————————————————————
## [CircuitPython 8.2.8] &nbsp;Built-in modules available:<br/>
https://circuitpython.org/board/yd_esp32_s3_n16r8/

_asyncio, _bleio, _pixelmap, adafruit_bus_device, adafruit_pixelbuf,
aesio, alarm, analogbufio, analogio, array, atexit, audiobusio,
audiocore, audiomixer, binascii, bitbangio, bitmaptools, board,
builtins, builtins.pow3, busio, busio.SPI, busio.UART, canio,
collections, countio, digitalio, displayio, dualbank, errno,
espcamera, espidf, espnow, espulp, fontio, framebufferio,
frequencyio, getpass, gifio, hashlib, i2ctarget, io, ipaddress,
json, keypad, keypad.KeyMatrix, keypad.Keys, keypad.ShiftRegisterKeys,
math, mdns, memorymap, microcontroller, msgpack, neopixel_write, nvm,
onewireio, os, os.getenv, ps2io, pulseio, pwmio, qrio, rainbowio,
random, re, rgbmatrix, rotaryio, rtc, sdcardio, select, sharpdisplay,
socketpool, ssl, storage, struct, supervisor, synthio, sys, terminalio,
time, touchio, traceback, ulab, usb_cdc, usb_hid, usb_midi, vectorio,
watchdog, wifi, zlib; Included frozen(?) modules: neopixel

———————————————————————————————————————————————————
# NOTES³:&nbsp; All Pin Names and Functions

The two tables below provide the Name and Function of the Pins on
both sides of this Dev board (J1 & J3).<br/>The Pin names are shown in
the ESP32-S3-DevKitC-1 image. The pin numbering matches the Board
Schematic (PDF).
```
【J1】 Left Side

--- ------- ------- ------------------------------------------------------------
No. Name    Type¹   Function
--- ------- ------- ------------------------------------------------------------
 1  3V3     P       3.3 V power supply
 2  3V3     P       3.3 V power supply
 3  RST     I       EN, RESET
 4  IO4     I/O/T   RTC_GPIO4,  GPIO4,  TOUCH4,  ADC1_CH3
 5  IO5     I/O/T   RTC_GPIO5,  GPIO5,  TOUCH5,  ADC1_CH4
 6  IO6     I/O/T   RTC_GPIO6,  GPIO6,  TOUCH6,  ADC1_CH5
 7  IO7     I/O/T   RTC_GPIO7,  GPIO7,  TOUCH7,  ADC1_CH6
 8  IO15    I/O/T   RTC_GPIO15, GPIO15, U0RTS,   ADC2_CH4, XTAL_32K_P
 9  IO16    I/O/T   RTC_GPIO16, GPIO16, U0CTS,   ADC2_CH5, XTAL_32K_N
10  IO17    I/O/T   RTC_GPIO17, GPIO17, U1TXD,   ADC2_CH6
11  IO18    I/O/T   RTC_GPIO18, GPIO18, U1RXD,   ADC2_CH7, CLK_OUT3
12  IO8     I/O/T   RTC_GPIO8,  GPIO8,  TOUCH8,  ADC1_CH7, SUBSPICS1, I²C SDA*
13  IO3     I/O/T   RTC_GPIO3,  GPIO3,  TOUCH3,  ADC1_CH2
14  IO46    I/O/T   DBG LOG*,   GPIO46  (INPUT ONLY)
15  IO9     I/O/T   RTC_GPIO9,  GPIO9,  TOUCH9,  ADC1_CH8, FSPIHD,  SUBSPIHD, I²C SCL*
16  IO10    I/O/T   RTC_GPIO10, GPIO10, TOUCH10, ADC1_CH9, FSPICS0, FSPIIO4,  SUBSPICS0
17  IO11    I/O/T   RTC_GPIO11, GPIO11, TOUCH11, ADC2_CH0, FSPID,   FSPIIO5,  SUBSPID
18  IO12    I/O/T   RTC_GPIO12, GPIO12, TOUCH12, ADC2_CH1, FSPICLK, FSPIIO6,  SUBSPICLK
19  IO13    I/O/T   RTC_GPIO13, GPIO13, TOUCH13, ADC2_CH2, FSPIQ,   FSPIIO7,  SUBSPIQ
20  IO14    I/O/T   RTC_GPIO14, GPIO14, TOUCH14, ADC2_CH3, FSPIWP,  FSPIDQS,  SUBSPIWP
21  5V      P       5V0 power supply    (IN-OUT jumper pad)
22  GND     G       Ground
```
```
【J3】 Right Side

--- ------- ------- ------------------------------------------------------------
No. Name    Type¹   Function
--- ------- ------- ------------------------------------------------------------
44  GND     G       Ground
43  TX      I/O/T   U0TXD*,    GPIO43, CLK_OUT1
42  RX      I/O/T   U0RXD*,    GPIO44, CLK_OUT2
41  IO1     I/O/T   RTC_GPIO1, GPIO1,  TOUCH1,   ADC1_CH0, -ALT- I²C SDA*
40  IO2     I/O/T   RTC_GPIO2, GPIO2,  TOUCH2,   ADC1_CH1, -ALT- I²C SCL*
39  IO42    I/O/T   MTMS*,     GPIO42
38  IO41    I/O/T   MTDI*,     GPIO41, CLK_OUT1
37  IO40    I/O/T   MTDO*,     GPIO40, CLK_OUT2
36  IO39    I/O/T   MTCK*,     GPIO39, CLK_OUT3, SUBSPICS1
35  IO38    I/O/T              GPIO38, FSPIWP,   SUBSPIWP
34  IO37    I/O/T   SPIDQS*,   GPIO37, FSPIQ,    SUBSPIQ,   OctalSPI ƒlash/PSRAM
33  IO36    I/O/T   SPIIO7*,   GPIO36, FSPICLK,  SUBSPICLK, OctalSPI ƒlash/PSRAM
32  IO35    I/O/T   SPIIO6*,   GPIO35, FSPID,    SUBSPID,   OctalSPI ƒlash/PSRAM
31  IO0     I/O/T   RTC_GPIO0, GPIO0
30  IO45    I/O/T              GPIO45
29  IO48    I/O/T   【RGB LED】, GPIO48, SPICLK_N, SUBSPICLK_N_DIFF
28  IO47    I/O/T              GPIO47, SPICLK_P, SUBSPICLK_P_DIFF
27  IO21    I/O/T  RTC_GPIO21, GPIO21
26  IO20    I/O/T  RTC_GPIO20, GPIO20, U1CTS, ADC2_CH9, CLK_OUT1, 【USB_D+】
25  IO19    I/O/T  RTC_GPIO19, GPIO19, U1RTS, ADC2_CH8, CLK_OUT2, 【USB_D-】
24  GND     G      Ground
23  GND     G      Ground

Type¹:  P: Power; G: Ground; I: Input; O: Output; T: High impedance.
```

———————————————————————————————————————————————————
### The ESP32-S3 peripherals include:
```
   20 Analog-to-Digital Converter (ADC) channels
    4 SPI interfaces
    3 UART interfaces
    2 I²C interfaces
    8 PWM output channel
    2 I²S interfaces
   14 Capacitive sensing GPIOs
```
Due to the ESP32 chip multiplexing feature,&nbsp; you can assign almost
any function to the GPIO of your choice.&nbsp; However, there are pins
with assigned functions by default.

### About this development board:

_Low-power performance_:<br/>
he ‘VCC-GND Studio, YD-ESP32-S3’ / ‘Binghe,
ESP32-S3 Core development board’ integrates 2.4GHz WiFi 802.11 b/g/n
and Bluetooth5(LE) dual-mode communication module,&nbsp; perfect for
Arduino Internet of Things (IoT) projects.

_Multiple Power Saving Modes_:<br/>
The ESP32-S3 development board supports
multiple low-power modes,&nbsp; which can be configured according to
different application scenarios to provide longer battery life.

_Simple programming and debugging_:<br/>
The ESP32-S3 module makes it easy
to program & burn in your ESP32-S3 board via dual USB Type-C ports,&nbsp;
with a choice of USB OTG or UART modes.

_Dual download modes_:<br/>
The ESP32-S3-DevKitC-1 module supports both USB
Direct connection download -and- USB to Serial port download,&nbsp; providing
more flexibility and convenience.

_Diverse connectivity options_:<br/>
The ESP32-S3-DevKitC-1 supports dual-mode 2.4GHZ WiFi 802.11 b/g/n and Bluetooth5.0(LE)
connectivity for a wide range of smart devices,&nbsp; making it ideal for Internet of Things
(IoT) applications.

```
                  Microcontroller: ESP32-S3
                             Chip: WROOM-1-N16R8, WROOM-1-N8R2
                              CPU: Dual-core Xtensa 32-bit LX7
                      Clock Speed: Up to 240 MHz
                     Flash Memory: 16MB (embedded)
                              RAM: 512KB
            Wireless Connectivity: WiFi 802.11 b/g/n, Bluetooth5.0(BLE)
                         I/O Pins: 34 GPIOs
Analog-to-Digital Converter (ADC): 12-bit resolution
                             UART: 3 UART interfaces
                              SPI: 2 SPI interfaces
                              I2C: 2 I2C interfaces
                              PWM: 16 channels
                     ADC Channels: 18
                Operating Voltage: 3.3V
                     Power Supply: USB-C or external 5V supply
                    USB Interface: USB-UART bridge for programming
                                   and debugging
```
———————————————————————————————————————————————————

Tools > USB CDC On Boot > Enabled<br/>
if using the USB OTG USB-C port.

———————————————————————————————————————————————————
<hr>
