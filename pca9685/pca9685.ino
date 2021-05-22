// PWM drive a LED using a PCA9685
//
//   - Example uses an ESP-WROOM32 ESP32 Board
//   - PCA9685 is connected through software I2C to pins D2 (SDA) and D4 (SCL)
//
// Dependencies: https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
//               Wire library
//
// Author Jan Delgado 05/2021, License MIT

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

constexpr auto LED_PIN = 15;        // PIN where LED is connected on PCA9685
constexpr auto I2C_ADDRESS = 0x40;  // Default I2C address of PCA9685

// Use Software I2C on Pins D2 and D4
constexpr auto SDA_PIN = 2;
constexpr auto SCL_PIN = 4;
TwoWire i2c = TwoWire(0);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(I2C_ADDRESS, i2c);

// ...or just use the defaults with SDA=D21, SCL=D22 (ESP32) and
// just construct a Adafruit_PWMServoDriver object like:
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver()

void setup() {
    i2c.begin(SDA_PIN, SCL_PIN);
    pwm.begin();
    pwm.setPWMFreq(1000);  
}

void loop() {
    static int count = 0;
    // setPin() sets PWM of given output to value in range [0,4095]
    pwm.setPin(LED_PIN, ((count % 256)<<4) | 15, false);
    count++;  
    delay(2);
}
