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


// bit buffer:
// - 100 measurements per second == 100 bits == 13 byte (4 bits unused)
#define BITS_PER_LINE     100
#define BYTE_PER_LINE     13
// - 16 seconds == 16 * 13 byte == 208 byte
#define NUM_LINES         16

#define NOT_SYNCED  255

uint8_t bitsums[100];
uint8_t convolution[100];
volatile uint8_t sampling_offset = NOT_SYNCED;
volatile bool recording;
volatile bool write_millis = false;

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

//  clearBuffer();

}

#if 0
void clearBuffer(void)
{
  recording = false;
  memset(recvd_bits, 0, sizeof(recvd_bits));
  isr_counter = 0;
  line_start = 0;
}

void drawBuffer(void)
{
  u8g2.firstPage();
  do {
    u8g2.drawXBM(0, 0, 100, 16, recvd_bits);
    drawSums(20);
    drawConvolution(42);
  } while ( u8g2.nextPage() );
}
#endif 

// -------------------------------------------------------------------
// Cyclic sampling ISR
// -------------------------------------------------------------------
ISR(TIMER1_COMPA_vect)
{
  Bits.sample();
}

#if 0
ISR(TIMER1_COMPA_vect)
{
  static bool is_recording = false;

  // synchronize recording with isr_counter overflow
  if (!is_recording && (isr_counter == 0) && (recording == true))
  {
    is_recording = true;
  }

  if (isr_counter == 0)
  {
    write_millis = true;
  }
  
  if (is_recording)
  {
    if (digitalRead(DCF77SIGNALPIN))
    {
      uint8_t byteidx = isr_counter >> 3;
      uint8_t bitidx = isr_counter & 0x07;
      recvd_bits[line_start + byteidx] |= 1 << bitidx;
    }
  }

  // keep track of our position within a one second period
  isr_counter++;
  if (isr_counter >= BITS_PER_LINE)
  {
    isr_counter = 0;
    line_start += BYTE_PER_LINE;
    if (line_start >= sizeof(recvd_bits))
    {
      is_recording = false;
      recording = false;
    }
  }

  // when synced, show our sampling window on the LED
  #if 1
  if (sampling_offset != NOT_SYNCED)
  {
    int8_t local_sampling_offset = sampling_offset;
    if (local_sampling_offset >= BITS_PER_LINE - 20)
    {
      local_sampling_offset -= BITS_PER_LINE;
    }
    int8_t sampling_end = local_sampling_offset + 20;
    int8_t local_isr_counter = isr_counter;
    if (local_isr_counter >= BITS_PER_LINE - 20)
    {
      local_isr_counter -= BITS_PER_LINE;
    }
    if ((local_isr_counter >= local_sampling_offset) && (local_isr_counter < sampling_end))
    {
      digitalWrite(DBGLEDPIN, HIGH);
    }
    else
    {
      digitalWrite(DBGLEDPIN, LOW);
    }
  }
  else
  {
    digitalWrite(DBGLEDPIN, LOW);
  }
  #endif

}
#endif

void calcSums(uint8_t buf)
{
  uint8_t *pBuffer = Bits.getBuffer(buf);
  
  for (uint8_t index = 0; index < BITS_PER_LINE; index++)
  {
    uint8_t byteidx = index >> 3;
    uint8_t bitidx = index & 0x07;
    bitsums[index] = 0;
    for (uint16_t line = 0; line < NUM_LINES * BYTE_PER_LINE; line+=BYTE_PER_LINE)
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
  for (uint8_t idx = 0; idx < BITS_PER_LINE; idx++)
  {
    uint16_t sum = 0;
    for (uint8_t convidx = 0; convidx < 10; convidx++)
    {
      uint8_t localidx = idx + convidx;
      if (localidx >= BITS_PER_LINE)
      {
        localidx -= BITS_PER_LINE;
      }
      sum += bitsums[localidx] << 1;
    }
    for (uint8_t convidx = 10; convidx < 20; convidx++)
    {
      uint8_t localidx = idx + convidx;
      if (localidx >= BITS_PER_LINE)
      {
        localidx -= BITS_PER_LINE;
      }
      sum += bitsums[localidx];
    }
    convolution[idx] = sum;
  }
}

void drawSums(uint16_t y)
{
  uint16_t base = y + 20;
  for (uint16_t x = 0; x < 100; x++)
  {
    u8g2.drawPixel(x, base - bitsums[x]);
  }
}

void drawConvolution(uint16_t y)
{
  uint16_t base = y + 20;
  for (uint16_t x = 0; x < 100; x++)
  {
    u8g2.drawPixel(x, base - (convolution[x] >> 4));
  }
}

uint8_t findConvolutionMax()
{
  uint16_t maxvalue = 0;
  uint8_t maxidx = 0;
  for (uint8_t idx = 0; idx < BITS_PER_LINE; idx++)
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
  static uint8_t lastBuffer = 0;

  uint8_t activeBuffer = Bits.getActiveBuffer();
  if (activeBuffer != lastBuffer)
  {
    calcSums(lastBuffer);
    convolute();
    uint8_t m = findConvolutionMax();
    uint8_t m_ = m+20;
    if (m_ >= 100)
      m_ -= 100;
    u8g2.firstPage();
    u8g2.setFont(u8g2_font_5x7_tf);
    do
    {
      u8g2.drawXBM(0, lastBuffer << 3, 100, 8, Bits.getBuffer(lastBuffer));
      drawConvolution(20);
      u8g2.setCursor(2,50);
      u8g2.print(m);
      u8g2.drawVLine(m, 17, 5);
      u8g2.drawVLine(m_, 17, 5);
      u8g2.drawPixel(m+1, 19);
      u8g2.drawPixel(m_-1, 19);
    } while ( u8g2.nextPage() );

   
    
    lastBuffer = activeBuffer;
  }
  
  #if 0
  recording = true;
  while (recording)
  {
    if (write_millis)
    {
      write_millis = false;
      //Serial.println(millis());
    }
  }
  //writeBuffer();
  calcSums();
  convolute();
  Serial.write('s');
  Serial.println(millis());
  drawBuffer();
  Serial.write('e');
  Serial.println(millis());
  uint8_t conv_max = findConvolutionMax();
  noInterrupts();
  sampling_offset = conv_max;
  interrupts();
  Serial.print("offset: ");
  Serial.println(sampling_offset, DEC);
  clearBuffer();
#endif
}

