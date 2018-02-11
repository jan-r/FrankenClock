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
#include "DCF77Decoder.h"
#include <Time.h>

// define DEBUG_PRINT to get some serial debugging output
//#define DEBUG_PRINT

#define DCF77_STATUS_MINUTES_OK     (1 << 0)
#define DCF77_STATUS_HOURS_OK       (1 << 1)
#define DCF77_STATUS_DATE_OK        (1 << 2)


DCF77Decoder::DCF77Decoder()
{
  reset();
}

void DCF77Decoder::reset()
{
  index = -1;
  bits[0] = 0;
  bits[1] = 0;
  valid[0] = 0;
  valid[1] = 1;

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
    // decoding sequence not started yet, only accept NONE (pause)
    if (value == DCF77_BIT_NONE)
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
    if (value == DCF77_BIT_0)
    {
      index++;
      valid[0] |= 1;
    }
    else
    {
      reset();
    }
  }
  else if (index == 20)
  {
    // bit 20 (start of time info) must always be 1
    if (value == DCF77_BIT_1)
    {
      bits[0] |= 1UL << 20;
      valid[0] |= 1UL << 20;
      index++;
    }
    else
    {
      reset();
    }
  }
  else
  {
    switch (value)
    {
      case DCF77_BIT_0:
      case DCF77_BIT_1:
      case DCF77_BIT_ERROR:
        if (index >= 32)
        {
          bits[1] |= (unsigned long)value << (index - 32);
          if (value != DCF77_BIT_ERROR)
          {
            valid[1] |= 1 << (index - 32);
          }
        }
        else if (index > 0)
        {
          bits[0] |= (unsigned long)value << index;
          if (value != DCF77_BIT_ERROR)
          {
            valid[0] |= 1 << index;
          }
        }
        index++;
        break;

      case DCF77_BIT_NONE:
      default:
        // start a new decoding cycle
        reset();
        break;
    }
  }

  if (index >= 59)
  {
    dataReady();
    reset();
  }

}

// ----------------------------------------------------------------------------
// Handle received data
// ----------------------------------------------------------------------------
void DCF77Decoder::dataReady()
{
  #ifdef DEBUG_PRINT
  Serial.write('!');
  #endif

  uint8_t datastatus = checkRcvdStream();

  #ifdef DEBUG_PRINT
  Serial.write(datastatus + 0x30);
  #endif

  if (datastatus)
  {
    // there was at least one valid chunk of information
  
    time_t currentTime = now();
    int hr = hour(currentTime);
    int mnt = minute(currentTime);
    int dy = day(currentTime);
    int mnth = month(currentTime);
    int yr   = year(currentTime);

    if (datastatus & DCF77_STATUS_HOURS_OK)
    {
      // extract hour
      unsigned long timebits = (bits[0] >> 21) | (bits[1] << 11);
      hr = ((timebits >> 8) & 0x0F) + 10 * ((timebits >> 12) & 0x03);
    }
    
    if (datastatus & DCF77_STATUS_MINUTES_OK)
    {
      // extract minute
      unsigned long timebits = (bits[0] >> 21) | (bits[1] << 11);
      mnt = (timebits & 0x0F) + 10 * ((timebits >> 4) & 0x07);
    }
  
    if (datastatus & DCF77_STATUS_DATE_OK)
    {
      // extract day
      dy = ((bits[1] >> 4) & 0xFU) + 10 * ((bits[1] >> 8) & 0x3);
      
      // extract month
      mnth = ((bits[1] >> 13) & 0xFU) + 10 * ((bits[1] >> 17) & 0x1);
  
      // extract year
      yr = ((bits[1] >> 18) & 0xFU) + 10 * ((bits[1] >> 22) & 0xFU) + 2000;
    }
    
    setTime(hr, mnt, 0, dy, mnth, yr);
    #ifdef DEBUG_PRINT
    Serial.write('*');
    #endif
  }
}

// ----------------------------------------------------------------------------
// Check the given 32 bit word for even parity
// ----------------------------------------------------------------------------
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


// ----------------------------------------------------------------------------
// Check which parts of the received stream were valid
// ----------------------------------------------------------------------------
uint8_t DCF77Decoder::checkRcvdStream()
{
  unsigned long bitsToCheck;
  unsigned long validCheck;
  uint8_t statusbits = 0;

  // -----------------------------------------------------------------
  // Parity checks
  // -----------------------------------------------------------------
  // minutes (bit 28:21)
  bitsToCheck = bits[0] & 0x1FE00000UL;
  validCheck  = valid[0] & 0x1FE00000UL;
  if (validCheck == 0x1FE00000UL)
  {
    // all bits were valid, continue with parity check
    if (checkParity(bitsToCheck))
    {
      statusbits |= DCF77_STATUS_MINUTES_OK;
    }
  }

  // hours (bit 35:29)
  bitsToCheck = (bits[0] & 0xE0000000UL) | (bits[1] & 0x0000000FUL);
  validCheck =  (valid[0] & 0xE0000000UL) | (valid[1] & 0x0000000FUL);
  if (validCheck == 0xE000000FUL)
  {
    // all bits were valid, continue with parity check
    if (checkParity(bitsToCheck))
    {
      statusbits |= DCF77_STATUS_HOURS_OK;
    }
  }
  
  // date (bit 58:36)
  bitsToCheck = bits[1] & 0x7FFFFF0UL;
  validCheck = valid[1] & 0x7FFFFF0UL;
  if (validCheck == 0x7FFFFF0UL)
  {
    // all bits were valid, continue with parity check
    if (checkParity(bitsToCheck))
    {
      statusbits |= DCF77_STATUS_DATE_OK;
    }
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
  return statusbits;
}


