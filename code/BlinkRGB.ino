/* BlinkRGB.ino
   ESP32-S3-WROOM-1 Dev / YD-ESP32-S3

 Demonstrate usage of the onboard RGB LED on some ESP dev boards.

 Calling digitalWrite(RGB_BUILTIN, HIGH) will use hidden RGB driver.
    
 neopixelWrite() demonstrates control of each color channel:
 void neopixelWrite(uint8_t pin, uint8_t Red, uint8_t Green, uint8_t Blue)

 WARNING: After using digitalWrite() to drive the RGB LED it will be 
 impossible to drive the same pin with normal HIGH/LOW levels.
*/

//#define RGB_BRIGHTNESS 64     // Change white brightness (max 255)

#define RGB_BUILTIN 48

void setup() {
  // No need to initialize the RGB LED
}

void loop() {
  neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,RGB_BRIGHTNESS,RGB_BRIGHTNESS); // White
  delay(1000);
  neopixelWrite(RGB_BUILTIN,RGB_BRIGHTNESS,0,0); // Red
  delay(1000);
  neopixelWrite(RGB_BUILTIN,0,RGB_BRIGHTNESS,0); // Green
  delay(1000);
  neopixelWrite(RGB_BUILTIN,0,0,RGB_BRIGHTNESS); // Blue
  delay(1000);
  neopixelWrite(RGB_BUILTIN,0,0,0); // Off / black
  delay(1000);
}
