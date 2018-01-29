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
  uint8_t recvdBits[2][BYTE_PER_SEC * NUM_SECONDS];
  uint8_t activeBuffer;
  uint8_t signalPin;
  uint8_t isrCounter;
  uint16_t lineStart;


public:
  Sampler(uint8_t SignalPin);

  void sample();
  
  uint8_t getActiveBuffer()
  {
    return activeBuffer;
  }

  uint8_t* getBuffer(uint8_t buf)
  {
    return &recvdBits[buf][0];
  }

};



#endif

