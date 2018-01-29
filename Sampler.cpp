#include "Sampler.h"


Sampler::Sampler(uint8_t SignalPin)
: activeBuffer(0), signalPin(SignalPin), isrCounter(0), lineStart(0)
{

}

void Sampler::sample()
{
  uint8_t sig = digitalRead(signalPin);
  digitalWrite(13, sig);

  if (sig)
  {
    uint8_t byteidx = isrCounter >> 3;
    uint8_t bitidx = isrCounter & 0x07;
    recvdBits[activeBuffer][lineStart + byteidx] |= 1 << bitidx;
  }

  // keep track of our position within a one second period
  isrCounter++;
  if (isrCounter >= BITS_PER_SEC)
  {
    isrCounter = 0;
    lineStart += BYTE_PER_SEC;
    if (lineStart >= BYTE_PER_SEC*NUM_SECONDS)
    {
      lineStart = 0;
      activeBuffer ^= 1;
      memset(&recvdBits[activeBuffer][0], 0, BYTE_PER_SEC*NUM_SECONDS);
    }
  }


}

