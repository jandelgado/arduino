/*
 * demo for tiny ESP32-C3 board with 0.42" OLED display using u8glib2.
 * The OLED is connected to I²C pins 5 (SDA) and 6 (SCL).
 * The on-board LED is connected to pin 8 and low-active.
 */

#include "U8g2lib.h"
#include "jled.h"
#include <Wire.h>
#include <driver/temp_sensor.h>

#define SDA_PIN 5
#define SCL_PIN 6

U8G2_SH1106_72X40_WISE_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
auto led = JLed(8).Breathe(4000).MinBrightness(20).LowActive().Forever();

void setup(void) {
  // initialize I²C
  Wire.begin(SDA_PIN, SCL_PIN);
  u8g2.begin();

  // Initialize temperature sensor
  temp_sensor_config_t temp_sensor = {
      .dac_offset = TSENS_DAC_DEFAULT,
      .clk_div = 6,
  };
  temp_sensor_set_config(temp_sensor);
  temp_sensor_start();
}

void loop(void) {
  static long last_update = 0;

  if (millis() - last_update > 1000) {
    u8g2.firstPage();
    do {
      float temp;
      temp_sensor_read_celsius(&temp);

      u8g2.setFont(u8g2_font_torussansbold8_8r);
      u8g2.drawFrame(0, 0, 72, 40);
      u8g2.drawStr(2, 10, "Hello");
      u8g2.drawStr(2, 18, "ESP32-C3");
      u8g2.drawStr(2, 26, (String("T=") + String(temp, 1)).c_str());
      u8g2.drawStr(2, 34, (String(last_update / 1000)).c_str());

    } while (u8g2.nextPage());
    last_update = millis();
  }

  led.Update();
  delay(1);
}
