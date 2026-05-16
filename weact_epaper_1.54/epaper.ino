// Example sketch for the WeAct Studio 1.54" epaper display
//
// derived from
// https://raw.githubusercontent.com/WeActStudio/WeActStudio.EpaperModule/refs/heads/master/Example/EpaperModuleTest_Arduino_ESP32/EpaperModuleTest_Arduino_ESP32.ino
//

#include <Fonts/FreeMonoBold9pt7b.h>
#include <GxEPD2_BW.h>

// ESP32 SCL(SCK)=18, SDA(MOSI)=23
#define CS_PIN (5)
#define BUSY_PIN (4)
#define RES_PIN (16)
#define DC_PIN (17)

// 1.54'' EPD Module
GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT>
    display(GxEPD2_154_D67(/*CS=5*/ CS_PIN, /*DC=*/DC_PIN, /*RES=*/RES_PIN,
                           /*BUSY=*/BUSY_PIN)); // GDEH0154D67 200x200, SSD1681

// embedded images image1_raw, image2_raw
#include "image1.h"
#include "image2.h"

const char HelloWorld[] = "Hello World!";
const char HelloWeACtStudio[] = "WeAct Studio";

void setup() {
  display.init(115200, true, 50, false);

  helloWorld();
  delay(1000);
  showImage(image1_raw);
  delay(5000);
  showImage(image2_raw);

  display.hibernate();
}


void showImage(const unsigned char* image) {
  display.setFullWindow();
  display.setRotation(2); // upside down (180°)
  display.firstPage();

  do {
    display.fillScreen(GxEPD_BLACK);

    // Draw bitmap at top-left corner
    display.drawBitmap(0, 0, image, 200, 200, GxEPD_WHITE);
  } while (display.nextPage());
}

void helloWorld() {
  display.setRotation(2);
  display.setFont(&FreeMonoBold9pt7b);
  display.setTextColor(GxEPD_BLACK);
  int16_t tbx, tby;
  uint16_t tbw, tbh;
  display.getTextBounds(HelloWorld, 0, 0, &tbx, &tby, &tbw, &tbh);
  // center the bounding box by transposition of the origin:
  uint16_t x = ((display.width() - tbw) / 2) - tbx;
  uint16_t y = ((display.height() - tbh) / 2) - tby;
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
    display.setCursor(x, y - tbh);
    display.print(HelloWorld);
    display.setTextColor(display.epd2.hasColor ? GxEPD_RED : GxEPD_BLACK);
    display.getTextBounds(HelloWeACtStudio, 0, 0, &tbx, &tby, &tbw, &tbh);
    x = ((display.width() - tbw) / 2) - tbx;
    display.setCursor(x, y + tbh);
    display.print(HelloWeACtStudio);
  } while (display.nextPage());
}

void loop() {
}
