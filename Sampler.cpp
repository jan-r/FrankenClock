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
#include "Sampler.h"

#define BUFFER_WAIT_SYNC    0
#define BUFFER_FILLING      1
#define BUFFER_FULL         2

#define SAMPLE_SETPOINT     40
#define MAX_CORRECTION     250

// define this to get a visual feedback on pin D13 (which is usually the on-board LED)
//#define ECHO_SIGNAL_ON_PIN_13

// define this to write the sampled bit length to the serial port
//#define PRINT_BITLENGTH

// define this to write a packed bitstream to the serial port
//#define PRINT_BITSTREAM

// Bit timing definitions
#define SAMPLER_BIT_NONE_MAX    1     //  10 ms
#define SAMPLER_BIT_ZERO_MIN    7     //  70 ms
#define SAMPLER_BIT_ZERO_MAX    11    // 110 ms
#define SAMPLER_BIT_ONE_MIN     18    // 180 ms

#ifdef PRINT_BITSTREAM
char hexchar[] = "0123456789ABCDEF";
#endif


Sampler::Sampler(uint8_t SignalPin, DCF77Decoder& decoder)
: signalPin(SignalPin), isrCounter(0), sampleLine(0), bufferState(BUFFER_WAIT_SYNC), currentMax(-1),
  correctionTicks(0), Decoder(decoder)
{

}

void Sampler::clearBuffer()
{
  sampleLine = 0;
  memset(recvdBits, 0, sizeof(recvdBits));
  bufferState = BUFFER_WAIT_SYNC;
}

void Sampler::sample()
{
  uint8_t sig = digitalRead(signalPin);

  #ifdef PRINT_BITSTREAM
  static uint8_t bitnum = 0;
  static uint8_t bits = 0;
  bits = (bits << 1) | sig;
  bitnum++;
  if (bitnum >= 4)
  {
    Serial.write(hexchar[bits]);
    bitnum = 0;
    bits = 0;
  }
  #endif

  #ifdef ECHO_SIGNAL_ON_PIN_13
  digitalWrite(13, sig);
  #endif

  // feed the bit decoder if our signal is stable
  if ((currentMax > 20) && (currentMax < 60))
  {
    if (isrCounter < currentMax)
    {
      bitCounter = 0;
    }
    else if (isrCounter < currentMax + 21)
    {
      bitCounter += sig;
    }
    else if (isrCounter == currentMax + 21)
    {
      if (bitCounter <= SAMPLER_BIT_NONE_MAX)
      {
        Decoder.nextBit(DCF77_BIT_NONE);
      }
      else if ((bitCounter >= SAMPLER_BIT_ZERO_MIN ) && (bitCounter <= SAMPLER_BIT_ZERO_MAX))
      {
        Decoder.nextBit(DCF77_BIT_0);
      }
      else if (bitCounter > SAMPLER_BIT_ONE_MIN)
      {
        Decoder.nextBit(DCF77_BIT_1);
      }
      else
      {
        Decoder.nextBit(DCF77_BIT_ERROR);
      }

      #ifdef PRINT_BITLENGTH
      Serial.println(bitCounter);
      #endif
    }
  }

  // still waiting for start of sampling period?
  if ((bufferState == BUFFER_WAIT_SYNC) && (isrCounter == 0))
  {
    // start sampling
    bufferState = BUFFER_FILLING;
  }
  
  if ((sig) && (bufferState == BUFFER_FILLING))
  {
    uint8_t byteidx = isrCounter >> 3;
    uint8_t bitidx = isrCounter & 0x07;
    recvdBits[sampleLine][byteidx] |= 1 << bitidx;
  }

  // keep track of our position within a one second period
  isrCounter++;
  if (isrCounter >= BITS_PER_SEC)
  {
    isrCounter = 0;
    sampleLine++;
    if (sampleLine >= NUM_SECONDS)
    {
      bufferState = BUFFER_FULL;
    }
  }
}

const uint8_t* Sampler::getBuffer()
{
  if (bufferState == BUFFER_FULL)
  {
    return &recvdBits[0][0];
  }
  else
  {
    return NULL;
  }
}

void Sampler::processBuffer()
{
  // sum up the bit values at each position over all seconds
  for (uint8_t index = 0; index < BITS_PER_SEC; index++)
  {
    uint8_t byteidx = index >> 3;
    uint8_t bitidx = index & 0x07;
    bitsums[index] = 0;
    for (uint8_t line = 0; line < NUM_SECONDS; line++)
    {
      if (recvdBits[line][byteidx] & (1 << bitidx))
      {
        bitsums[index]++;
      }
    }
  }

  // convolute the summed values with the expected signal
  uint8_t maxidx = 0;
  uint16_t maxvalue = 0;
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
    if (sum > maxvalue)
    {
      maxidx = idx;
      maxvalue = sum;
    }
  }
  currentMax = maxidx;
  PLL();
}

void Sampler::PLL()
{
  int16_t delta = currentMax - SAMPLE_SETPOINT;
  int16_t prop = delta << 3;
  if (prop > MAX_CORRECTION)
  {
    prop = MAX_CORRECTION;
  }
  else if (prop < -MAX_CORRECTION)
  {
    prop = -MAX_CORRECTION;
  }
  correctionTicks = prop;
}

