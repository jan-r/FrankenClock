The FrankenClock
================

A DCF77 desktop clock project with thermometer and hygrometer.

Introduction:
-------------

The clock hardware is based on an Arduino Nano and other parts I had lying
around or salvaged from old stuff.

It does not make any sense to build this project, unless you want to learn
about the DCF77 protocol or any of the other components I used.

Project status:
---------------

* Bit sampler works, including a PLL that stabilizes the sampling window
* Temperature and humidity measurement works
* Shows debug output until the first sync, then switches to time/temperature/humidity

* No error correction yet, only error detection
* Bit length tolerance is far too wide, need to take some measurements under bad conditions
  to fine-tune this

Components:
-----------

* Arduino Nano (or compatible)
* DHT22 sensor
* DCF77 module
* OLED display, SSD1306-compatible, I2C, 128x64

Hardware setup:
---------------

* Arduino D2:           DHT22 "data" pin
* Arduino D3:           DHT22 "VCC" pin
* Arduino D4:           DCF77 module "PON" pin (power mode)
* Arduino D5:           DCF77 module "TCO" pin (time signal)
* Arduino A4 (I2C SDA): SDA pin of display
* Arduino A5 (I2C SCL): SCL pin of display

The DHT22 sensor is supplied from an Arduino output pin. The maximum current
drawn by the sensor is around 1.5 mA, so the pin can easily supply the chip.
This way the sensor can be switched off to save power and avoid self-heating
of the sensor. The sensor is enabled 1s before a measurement is taken and
switched off after the measurement.

The DCF77 module was harvested from an old alarm clock. It operates on any
voltage between 3V and 5V. The "PON" pin controlls the module's power mode:
pulling the signal LOW will enable the module. Pulling it HIGH will disable
the module and save power. With PON set to low, the module will provide the
DCF77 time signal on its "TCO" pin. The time signal is transmitted as a bit
stream with one bit per second and a length of 58 or 59 bits. A "0" bit is
transmitted as a pulse with a width of around 100 ms, and a "1" is a pulse with
a width of around 200 ms.

Required software libraries:
----------------------------

* Install from the integrated Arduino library manager:
  * Adafruit unified sensor library
  * Adafruit DHT unified library
  * u8g2 library

Thanks:
-------

A big "thank you" goes to G. Lay for his thoughts about DCF77 error correction.
See http://www.gjlay.de/software/dcf77/konzept.html (German only). The DCF77
decoding class was heavily inspired by this page.


