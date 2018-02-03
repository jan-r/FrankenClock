#include "DCF77Decoder.h"
#include <Time.h>

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

  if (checkRcvdStream())
  {
    unsigned long timebits;
    int hr, mnt;
    int dy, mnth, yr;

    // extract hour
    timebits = (bits[0] >> 21) | (bits[1] << 11);
    hr = ((timebits >> 8) & 0x0F) + 10 * ((timebits >> 12) & 0x03);

    // extract minute
    timebits = (bits[0] >> 21) | (bits[1] << 11);
    mnt = (timebits & 0x0F) + 10 * ((timebits >> 4) & 0x07);

    // extract day
    dy = ((bits[1] >> 4) & 0xFU) + 10 * ((bits[1] >> 8) & 0x3);
    
    // extract month
    mnth = ((bits[1] >> 13) & 0xFU) + 10 * ((bits[1] >> 17) & 0x1);

    // extract year
    yr = ((bits[1] >> 18) & 0xFU) + 10 * ((bits[1] >> 22) & 0xFU) + 2000;
    
    setTime(hr, mnt, 0, dy, mnth, yr);
    Serial.write('*');
  }
}

bool DCF77Decoder::checkParity(unsigned long bitsToCheck)
{
  char parity = 0;
  for (char i = 0; i < 32; i++)
  {
    parity ^= bitsToCheck & 1;
    bitsToCheck >>= 1;
  }
  return (parity == 0);
}

bool DCF77Decoder::checkRcvdStream()
{
  unsigned long bitsToCheck;
  bool isOk = true;

  // -----------------------------------------------------------------
  // Parity checks
  // -----------------------------------------------------------------
  // minutes (bit 28:21)
  bitsToCheck = bits[0] & 0x1FE00000UL;
  if (!checkParity(bitsToCheck))
  {
    isOk = false;
  }

  // hours (bit 35:29)
  bitsToCheck = (bits[0] & 0xE0000000UL) | (bits[1] & 0x0000000FUL);
  if (!checkParity(bitsToCheck))
  {
    isOk = false;
  }

  // date (bit 58:36)
  bitsToCheck = bits[1] & 0x7FFFFF0UL;
  if (!checkParity(bitsToCheck))
  {
    isOk = false;
  }
#if 0
  // -----------------------------------------------------------------
  // Sanity checks
  // -----------------------------------------------------------------
  int val = minute();
  if ((val < 0) || (val > 59))
  {
    isOk = false;
  }

  val = hour();
  if ((val < 0) || (val > 23))
  {
    isOk = false;
  }

  val = day();
  if ((val < 1) || (val > 31))
  {
    isOk = false;
  }

  val = month();
  if ((val < 1) || (val > 12))
  {
    isOk = false;
  }
#endif
  return isOk;
}


