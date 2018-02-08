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
#ifndef SAMPLER_H
#define SAMPLER_H

#include <Arduino.h>
#include "DCF77Decoder.h"

// bit buffer:
// - 100 measurements per second == 100 bits == 13 byte (4 bits unused)
#define BITS_PER_SEC      100
#define BYTE_PER_SEC      13
// - 8 seconds == 8 * 13 byte == 104 byte
#define NUM_SECONDS       8

class Sampler
{
private:
  uint8_t recvdBits[NUM_SECONDS][BYTE_PER_SEC];
  uint8_t activeBuffer;
  uint8_t signalPin;
  uint8_t isrCounter;
  uint8_t sampleLine;
  uint8_t bufferState;
  uint8_t bitCounter;

  uint8_t bitsums[BITS_PER_SEC];
  uint8_t convolution[BITS_PER_SEC];
  uint8_t currentMax;

  void PLL(void);
  int16_t correctionTicks;

  DCF77Decoder& Decoder;

public:
  Sampler(uint8_t SignalPin, DCF77Decoder& decoder);

  void sample();
  const uint8_t* getBuffer();
  void clearBuffer();
  void processBuffer();

  const uint8_t* getConvolution() const
  {
    return convolution;
  }

  uint8_t getCurrentMax() const
  {
    return currentMax;
  }

  int16_t getCorrectionTicks()
  {
    return correctionTicks;
  }
};



#endif

