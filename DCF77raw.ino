#include <Arduino.h>
#include <U8g2lib.h>

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

Sampler Bits(DCF77SIGNALPIN);

// ----------------------------------------------------------------------------
// Display
// ----------------------------------------------------------------------------
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, DISP_CLOCK, DISP_DATA, U8X8_PIN_NONE);
#define DISPLAY_RES_X   128
#define DISPLAY_RES_Y   64


uint8_t bitsums[100];
uint8_t convolution[100];


// ----------------------------------------------------------------------------
// Setup
// ----------------------------------------------------------------------------
void setup()
{
  u8g2.begin();

  
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
  OCR1A = 20000;
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


void calcSums(const uint8_t *pBuffer)
{
  for (uint8_t index = 0; index < BITS_PER_SEC; index++)
  {
    uint8_t byteidx = index >> 3;
    uint8_t bitidx = index & 0x07;
    bitsums[index] = 0;
    for (uint16_t line = 0; line < NUM_SECONDS * BYTE_PER_SEC; line+=BYTE_PER_SEC)
    {
      if (pBuffer[line + byteidx] & (1 << bitidx))
      {
        bitsums[index]++;
      }
    }
  }
}

void convolute(void)
{
  for (uint8_t idx = 0; idx < BITS_PER_SEC; idx++)
  {
    uint16_t sum = 0;
    for (uint8_t convidx = 0; convidx < 10; convidx++)
    {
      uint8_t localidx = idx + convidx;
      if (localidx >= BITS_PER_SEC)
      {
        localidx -= BITS_PER_SEC;
      }
      sum += bitsums[localidx] << 1;
    }
    for (uint8_t convidx = 10; convidx < 20; convidx++)
    {
      uint8_t localidx = idx + convidx;
      if (localidx >= BITS_PER_SEC)
      {
        localidx -= BITS_PER_SEC;
      }
      sum += bitsums[localidx];
    }
    convolution[idx] = sum;
  }
}

void drawSums(uint16_t y)
{
  uint16_t base = y + 20;
  for (uint16_t x = 0; x < BITS_PER_SEC; x++)
  {
    u8g2.drawPixel(x, base - bitsums[x]);
  }
}

void drawConvolution(uint16_t y)
{
  uint16_t base = y + 20;
  for (uint16_t x = 0; x < BITS_PER_SEC; x++)
  {
    u8g2.drawPixel(x, base - (convolution[x] >> 4));
  }
}

uint8_t findConvolutionMax()
{
  uint16_t maxvalue = 0;
  uint8_t maxidx = 0;
  for (uint8_t idx = 0; idx < BITS_PER_SEC; idx++)
  {
    if (convolution[idx] > maxvalue)
    {
      maxvalue = convolution[idx];
      maxidx = idx;
    }
  }
  return maxidx;
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
    calcSums(buf);
    convolute();
    uint8_t m = findConvolutionMax();
    uint8_t m_ = m+20;
    if (m_ >= 100)
      m_ -= 100;
    u8g2.firstPage();
    u8g2.setFont(u8g2_font_5x7_tf);
    do
    {
      u8g2.drawXBM(0, 0, 100, 8, buf);
      drawConvolution(20);
      u8g2.setCursor(2,50);
      u8g2.print(m);
      u8g2.drawVLine(m, 17, 5);
      u8g2.drawVLine(m_, 17, 5);
      u8g2.drawPixel(m+1, 19);
      u8g2.drawPixel(m_-1, 19);
    } while ( u8g2.nextPage() );
    Bits.clearBuffer();
  }
  
}

