// Heltec Lora Wifi 32 OLED Demo

#include <Arduino.h>
#include <U8x8lib.h>
#include <SPI.h>


U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

void setup(void)
{
  u8x8.begin();

  u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);    
  u8x8.clear();

  u8x8.setCursor(0,0);
  u8x8.print("Heltec Wifi");
  u8x8.setCursor(0,1);
  u8x8.print("Lora 32");
}


void loop(void)
{
  delay(1000);
}

