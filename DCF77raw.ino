#include <Arduino.h>
#include <U8g2lib.h>

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

uint8_t recvd_bits[BYTE_PER_LINE * NUM_LINES];
uint8_t bitsums[100];

uint16_t line_start;
uint8_t bit_in_line;
volatile bool recording;

// ----------------------------------------------------------------------------
// Setup
// ----------------------------------------------------------------------------
void setup() {
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

  clearBuffer();

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = (1 << WGM12); // enable CTC mode
  TCNT1 = 0;
  OCR1A = 20000;
  TCCR1B |= (1 << CS11); // prescaler *8
  TIMSK1 |= (1 << OCIE1A);  // enable compare interrupt
  interrupts();
}

void clearBuffer(void)
{
  recording = false;
  memset(recvd_bits, 0, sizeof(recvd_bits));
  bit_in_line = 0;
  line_start = 0;
}

void drawBuffer(void)
{
  u8g2.firstPage();
  do {
    u8g2.drawXBM(0, 0, 100, 16, recvd_bits);
    drawSums(20);
  } while ( u8g2.nextPage() );
}

ISR(TIMER1_COMPA_vect)
{
  if (recording)
  {
    if (digitalRead(DCF77SIGNALPIN))
    {
      uint8_t byteidx = bit_in_line >> 3;
      uint8_t bitidx = bit_in_line & 0x07;
      recvd_bits[line_start + byteidx] |= 1 << bitidx;
    }
    bit_in_line++;
    if (bit_in_line >= BITS_PER_LINE)
    {
      bit_in_line = 0;
      line_start += BYTE_PER_LINE;
      if (line_start >= sizeof(recvd_bits))
      {
        recording = false;
      }
    }
    //digitalWrite(DBGLEDPIN, HIGH);
    //Serial.write('1');
    
  }
  //else
  {
    //digitalWrite(DBGLEDPIN, LOW);
    //Serial.write('0');
  }
}

void calcSums(void)
{
  for (uint8_t index = 0; index < BITS_PER_LINE; index++)
  {
    uint8_t byteidx = index >> 3;
    uint8_t bitidx = index & 0x07;
    bitsums[index] = 0;
    for (uint16_t line = 0; line < NUM_LINES * BYTE_PER_LINE; line+=BYTE_PER_LINE)
    {
      if (recvd_bits[line + byteidx] & (1 << bitidx))
      {
        bitsums[index]++;
      }
    }
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
void loop() {
  recording = true;
  while (recording)
  {
  }
  writeBuffer();
  calcSums();
  drawBuffer();
  clearBuffer();
  // put your main code here, to run repeatedly:
}

