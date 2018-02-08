/*
 * This file is part of the FrankenClock project. It is licensed under the
 * BSD 3-clause license.
 *
 * Copyright 2018 Jan Reucker
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <Arduino.h>
#include <U8g2lib.h>
#include <Time.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include "Sampler.h"

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif


// ----------------------------------------------------------------------------
// Hardware setup
// ----------------------------------------------------------------------------
#define DHTPIN            2         // Pin which is connected to the DHT sensor.
#define DHTPOWERPIN       3         // Pin which supplies the DHT sensor with power.
#define DCF77POWERPIN     4         // Pin to control power of the DCF77 module (low == on)
#define DCF77SIGNALPIN    5         // Pin which connects to the DCF77 time signal 
#define DBGLEDPIN         13
#define DISP_DATA         A4        // HW I2C data line to display
#define DISP_CLOCK        A5        // HW I2C clock line to display

#define SERIAL_BAUDRATE   115200

#define OCR1A_RELOAD_DEFAULT    20000

uint16_t OCR1A_ReloadValue = OCR1A_RELOAD_DEFAULT;

DCF77Decoder Decoder;
Sampler Bits(DCF77SIGNALPIN, Decoder);

// ----------------------------------------------------------------------------
// Display
// ----------------------------------------------------------------------------
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, DISP_CLOCK, DISP_DATA, U8X8_PIN_NONE);
#define DISPLAY_RES_X   128
#define DISPLAY_RES_Y   64

// ----------------------------------------------------------------------------
// DHT22 sensor
// ----------------------------------------------------------------------------
#define DHTTYPE           DHT22     // DHT 22 (AM2302)
DHT_Unified dht(DHTPIN, DHTTYPE);

#define DEGREE '\xb0'               // degree symbol
float fCurrentTemp = 99.9f;
float fCurrentHumidity = 99.9f;

// calibration
#define REFERENCE_1     7.6f
#define VALUE_1         7.0f
#define REFERENCE_2     29.1f
#define VALUE_2         30.4f
float calibrationFactor = 1.0f;
float calibrationOffset = 0.0f;


// ----------------------------------------------------------------------------
// Setup
// ----------------------------------------------------------------------------
void setup()
{
  // initialize display
  u8g2.begin();

  // power up and initialize DHT22 sensor
  pinMode(DHTPOWERPIN, OUTPUT);
  digitalWrite(DHTPOWERPIN, HIGH);
  delay(1000);
  calibrate(REFERENCE_1, VALUE_1, REFERENCE_2, VALUE_2);
  dht.begin();

  
  // initialize serial command interface
  Serial.begin(SERIAL_BAUDRATE);
  pinMode(DCF77POWERPIN, OUTPUT);
  pinMode(DCF77SIGNALPIN, INPUT);
  pinMode(DBGLEDPIN, OUTPUT);    // debugging LED

  digitalWrite(DCF77POWERPIN, HIGH);
  digitalWrite(DBGLEDPIN, LOW);
  delay(1000);
  digitalWrite(DCF77POWERPIN, LOW);

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = (1 << WGM12); // enable CTC mode
  TCNT1 = 0;
  OCR1A = OCR1A_ReloadValue;
  TCCR1B |= (1 << CS11); // prescaler *8
  TIMSK1 |= (1 << OCIE1A);  // enable compare interrupt
  interrupts();
}

// -------------------------------------------------------------------
// Cyclic sampling ISR
// -------------------------------------------------------------------
ISR(TIMER1_COMPA_vect)
{
  Bits.sample();
}

void drawConvolution(uint16_t y)
{
  uint16_t base = y + 20;
  const uint8_t *pConvolution = Bits.getConvolution();
  for (uint16_t x = 0; x < BITS_PER_SEC; x++)
  {
    u8g2.drawPixel(x, base - (*(pConvolution + x) >> 4));
  }
}

#if 0
void writeBuffer(void)
{
  uint8_t *ptr = recvd_bits;
  Serial.println();
  for (uint8_t line = 0; line < NUM_LINES; line++)
  {
    for (uint8_t byteidx = 0; byteidx < BYTE_PER_LINE; byteidx++)
    {
      Serial.print("0x");
      Serial.print(*ptr++, HEX);
      Serial.write(',');
    }
    Serial.write('\n');
  }
}
#endif

// -------------------------------------------------------------------
// Main loop
// -------------------------------------------------------------------
void loop()
{
  const uint8_t *buf = Bits.getBuffer();
  
  if (buf != NULL)
  {
    Bits.processBuffer();
    uint8_t m = Bits.getCurrentMax();
    uint8_t m_ = m+20;
    if (m_ >= 100)
      m_ -= 100;
    OCR1A_ReloadValue = OCR1A_RELOAD_DEFAULT + Bits.getCorrectionTicks();
    noInterrupts();
    OCR1A = OCR1A_ReloadValue;
    interrupts(); 
    u8g2.firstPage();
    u8g2.setFont(u8g2_font_5x7_tf);
    do
    {
      u8g2.drawXBM(0, 0, 100, 8, buf);
      drawConvolution(20);
      u8g2.setCursor(2,50);
      u8g2.print(m);
      //u8g2.setCursor(40,50);
      //u8g2.print(OCR1A_ReloadValue);
      u8g2.drawVLine(m, 17, 5);
      u8g2.drawVLine(m_, 17, 5);
      u8g2.drawPixel(m+1, 19);
      u8g2.drawPixel(m_-1, 19);

      time_t t = now();
      u8g2.setCursor(70, 50);
      u8g2.print(hour(t));
      u8g2.print(":");
      if (minute(t) < 10)
      {
        u8g2.print("0");
      }
      u8g2.print(minute(t));

      u8g2.setCursor(2, 62);
      u8g2.print(String(fCurrentTemp, 1));
      u8g2.print(DEGREE);
      u8g2.print("C   ");
      u8g2.print(String(fCurrentHumidity, 0));
      u8g2.print("%");
    } while ( u8g2.nextPage() );
    Bits.clearBuffer();
  }
  updateSensors(millis());
}

// ----------------------------------------------------------------------------
// Calculate calibration values calibrationFactor and calibrationOffset from
// two measured values
// ref1/ref2: reference temperatures
// val1/val2: measured values at the given temperature
// ----------------------------------------------------------------------------
void calibrate(float ref1, float val1, float ref2, float val2)
{
  calibrationFactor = (ref2 - ref1) / (val2 - val1);
  calibrationOffset = (val2 * ref1 - val1 * ref2) / (val2 - val1);
}

// ----------------------------------------------------------------------------
// Fetch the current sensor values and update the global variables
// fCurrentTemp and fCurrentHumidity.
// ----------------------------------------------------------------------------
void updateSensors(unsigned long currentTime)
{
  enum {POWERING_UP, MEASURING, SLEEPING};
  static unsigned long lastStateChange = 0;
  static char state = MEASURING;
  sensors_event_t event;

  // sensor state machine:
  // - power up the sensor for 1000 ms to stabilize
  // - take a measurement
  // - power down the sensor for 9000 ms
  switch (state)
  {
    case MEASURING:
      dht.temperature().getEvent(&event);
      if (!isnan(event.temperature))
      {
        fCurrentTemp = event.temperature * calibrationFactor + calibrationOffset;
      }
      dht.humidity().getEvent(&event);
      if (!isnan(event.relative_humidity)) {
        fCurrentHumidity = event.relative_humidity;
      }
//      trackHistory(currentTime);
      // switch off the sensor
      digitalWrite(DHTPOWERPIN, LOW);
      state = SLEEPING;
      lastStateChange = currentTime;
      break;

    case SLEEPING:
      if ((currentTime - lastStateChange) >= 9000)
      {
        lastStateChange = currentTime;
        state = POWERING_UP;
      }
      break;

    case POWERING_UP:
    default:
      // enable sensor
      digitalWrite(DHTPOWERPIN, HIGH);
      if ((currentTime - lastStateChange) >= 1000)
      {
        lastStateChange = currentTime;
        state = MEASURING;
      }
      break;
  }
}


