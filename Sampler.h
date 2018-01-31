#ifndef SAMPLER_H
#define SAMPLER_H

#include <Arduino.h>

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


public:
  Sampler(uint8_t SignalPin);

  void sample();
  const uint8_t* getBuffer();
  void clearBuffer();

};



#endif

