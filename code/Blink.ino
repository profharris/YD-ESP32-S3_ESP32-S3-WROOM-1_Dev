/* Blink.ino
   https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  Serial.print('*');
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED On 
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED Off 
  delay(1000);                      // wait for a second
}

/*******************************************************************
SERIAL MONITOR:         [Both NL & CR  ⤋] [15200 baud  ⤋]

ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x1 (POWERON),boot:0x8 (SPI_FAST_FLASH_BOOT)
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce2820,len:0x1188
load:0x403c8700,len:0x4
load:0x403c8704,len:0xbf0
load:0x403cb700,len:0x30e4
entry 0x403c88ac
************************************

*******************************************************************/
