#include "DCF77Decoder.h"

#define DEBUG_PRINT

DCF77Decoder::DCF77Decoder()
{
  reset();
}

void DCF77Decoder::reset()
{
  index = -1;
  bits[0] = 0;
  bits[1] = 0;

  #ifdef DEBUG_PRINT
  Serial.write('\n');
  #endif
}

void DCF77Decoder::nextBit(uint8_t value)
{
  #ifdef DEBUG_PRINT
  if (value < 10)
  {
    Serial.write(value + 0x30);
  }
  #endif

  if (index == -1)
  {
    // decoding sequence not started yet, only accept NO_BIT
    if (value == NO_BIT)
    {
      // start of cycle
      index = 0;

      #ifdef DEBUG_PRINT
      Serial.write('S');
      #endif
    }
  }
  else if (index == 0)
  {
    // first bit must always be 0, otherwise reception was disturbed
    if (value == 0)
    {
      index++;
    }
    else
    {
      reset();
    }
  }
  else if (index == 20)
  {
    // bit 20 (start of time info) must always be 1
    if (value == 1)
    {
      bits[0] |= 1UL << 20;
      index++;
    }
    else
    {
      reset();
    }
  }
  else
  {
    if (value != NO_BIT)
    {
      if (index >= 32)
      {
        bits[1] |= (unsigned long)value << (index - 32);
      }
      else if (index > 0)
      {
        bits[0] |= (unsigned long)value << index;
      }
      index++;  
    }
    else
    {
      reset();
    }
  }

  if (index >= 59)
  {
    dataReady();
    reset();
  }

}

void DCF77Decoder::dataReady()
{
  #ifdef DEBUG_PRINT
  Serial.write('!');
  #endif
}

