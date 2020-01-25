// sipseed longan nano demo - cycle through builtin LEDs
// Jan Delgado 01-2020
//
#include <Arduino.h>

void setup() {
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
}

void loop() {
    // builtin LEDs are low-active
    static PinStatus levels[] = {LOW,HIGH,HIGH};
    static auto level = 0;

    digitalWrite(LED_RED, levels[level%3]);
    digitalWrite(LED_GREEN, levels[(level+1)%3]);
    digitalWrite(LED_BLUE, levels[(level+2)%3]);
    level++;

    delay(500);
}

