/* 
 * Demo for WEMOS "D-DUINO" w/ I2C SSD1306 based 128x64 OLED module using u8glib2.
 */

#include "U8g2lib.h"
#include <pins_arduino.h>

// connected via I2C D1-SDA, D2-SCL
U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, D2 /* SCL */, D1 /* SDA */);

void setup(void) {
  u8g2.begin();
}

/* draw something on the display with the `firstPage()`/`nextPage()` loop*/
void loop(void) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_torussansbold8_8r);
    u8g2.drawStr(0,20,"WEMOS D-DUINO");
    u8g2.drawStr(0,40,"SSD1306 (I2C)");
  } while ( u8g2.nextPage() );
  delay(1000);
}

