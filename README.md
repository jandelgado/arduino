# arduino

Arduino libraries and sketches.


<!-- vim-markdown-toc GFM -->

* [Libraries](#libraries)
    * [JLed](#jled)
    * [log4arduino](#log4arduino)
* [Sketches](#sketches)
    * [Interfacing SSD1306 based OLED displays (SPI)](#interfacing-ssd1306-based-oled-displays-spi)
    * [Colorduino RGB matrix driver](#colorduino-rgb-matrix-driver)
    * [CJMCU-8x8 RGB matix](#cjmcu-8x8-rgb-matix)

<!-- vim-markdown-toc -->

## Libraries

### JLed

JLed is an Arduino library to control LEDs. It uses a non-blocking approach and
can control LEDs in simple (on/off) and complex (blinking, breathing) ways in a
time-driven manner.

<img alt="jled" width=256 src="images/jled.gif">

* https://github.com/jandelgado/jled

### log4arduino

A lightweight, no-frills logging library for Arduino & friends.

```c++
LOG("hello, log4arduino.");
LOG("use %s formatting: %d %c %d %c %d", "printf", 9, '+', 1, '=', 10);
```

* https://github.com/jandelgado/log4arduino

## Sketches

### Interfacing SSD1306 based OLED displays (SPI)

<img alt="ssd1306" width=256 src="images/ssd1306.jpg">

* [ssd1306_sample_adafruit](ssd1306_sample_adafruit) - sample code for ssd1306 based oled display. see [wiki](http://github.com/jandelgado/arduino/wiki/SSD1306-based-OLED-connected-to-Arduino) for detailed information
* [ssd1306_sample_u8g](ssd1306_sample_u8g) - sample code for ssd1306 based oled display. see [wiki](http://github.com/jandelgado/arduino/wiki/SSD1306-based-OLED-connected-to-Arduino) for detailed information


### Colorduino RGB matrix driver

Driver for 8x8 RGB LED matrix.

<img alt="Colorduino" width=256 src="images/colorduino.JPG">

* see [wiki](https://github.com/jandelgado/arduino/wiki/Colorduino) for detailed information


### CJMCU-8x8 RGB matix

An 8x8 LED RGB matrix based on WS2812 "NeoPixels".

<img alt="CJMCU-8*8" width=256 src="images/cjmcu-8x8/cjmcu.jpg">

* see [README of example sketch](cjmcu_8x8_sample/README.md) for detailed information.
