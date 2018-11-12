/* 
 * Demo for ESP32 TTGO board w/ I2C SSD1306 based 128x64 OLED module 
 * using u8glib2 in full screen mode.
 * Jan Delgado 11-2018
 */

#include "U8g2lib.h"
#include <pins_arduino.h>

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, 4 /* SCL */, 5 /* SDA */);

void setup(void) {
  u8g2.begin();
}

void loop(void) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_torussansbold8_8r);
  u8g2.drawStr(0,20,"ESP32 TTGO");
  u8g2.drawStr(0,40,"SSD1306 (I2C)");
  u8g2.sendBuffer();
  delay(1000);
}

