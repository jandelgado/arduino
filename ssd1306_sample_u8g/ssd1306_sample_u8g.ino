/* 
 * Demo for SSD1306 based 128x64 OLED module using u8glib.
 * See https://github.com/pacodelgado/arduino/wiki/SSD1306-based-OLED-connected-to-Arduino 
 * for more information.
 *
 *  ---------------------------------------------------------------------
 * U8glib library copyright notice:
 * Universal 8bit Graphics Library, http://code.google.com/p/u8glib/
 * Copyright (c) 2012, olikraus@gmail.com
 * All rights reserved.
 */

#include "U8glib.h"

/* Create an instance for the SSD1306 OLED display in SPI mode 
 * connection scheme: 
 *   D0=sck=Pin 12 
 *   D1=mosi=Pin 11 
 *   CS=Pin 8 
 *   DC=A0=Pin 9
 *   Reset=Pin 10
 */
U8GLIB_SSD1306_128X64 u8g(12, 11, 8, 9, 10);

void setup() 
{
  /* nothing to do here */
}

void loop() 
{
  u8g.firstPage();  
  
  /* Keep looping until finished drawing screen */
  do 
  {
    int steps = 16;
    int dx = 128/steps;
    int dy = 64/steps;
    int y = 0;
    for(int x=0; x<128; x+=dx) {
        u8g.drawLine(x, 0, 127, y);
        u8g.drawLine(127-x, 63, 0, 63-y);
       y+=dy;     
    }
      
  } while(u8g.nextPage());
}
