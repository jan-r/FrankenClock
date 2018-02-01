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

