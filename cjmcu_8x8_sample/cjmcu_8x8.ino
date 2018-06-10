// Demo for CJMCU-8*8 LED RGB Matrix. Demo assumes CJMCU connected
// to an Arduino nano on PIN D9
// Jan Delgado 2018
#include <FastLED.h>

// LED configutration
#define LED_TYPE WS2812B   // type of LED in use. CJMCU works with
                           // WS2811,WS2812,WS2812B (compatible)

constexpr auto LED_DT = 9; // Pin WS2812 Data in (DIN) is connected to
constexpr auto COLOR_ORDER = GRB;  
constexpr auto NUM_LEDS = 64;
constexpr auto SPEED_MS = 50;

CRGB leds[NUM_LEDS];

void setup() {

    LEDS.addLeds<LED_TYPE, LED_DT, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(128);

    // FastLED 2.1 Power management set at 5V, 500mA
    set_max_power_in_volts_and_milliamps(5, 500);
    CRGB cols[] = {CRGB::Red, CRGB::Green, CRGB::Blue};

    for(auto c : cols) {
      for(auto i = 0; i<NUM_LEDS; i++ ) {
        leds[i] = c;
      }
      FastLED.show();
      delay(1000);
    }
}

void loop() {
    static int hue = 0;
    static int delta_hue = 4;

    EVERY_N_MILLISECONDS(SPEED_MS) {
        fill_rainbow(leds, NUM_LEDS, hue++, delta_hue);
    }
    show_at_max_brightness_for_power();
}
