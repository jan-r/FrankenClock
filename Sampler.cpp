#include "Sampler.h"

#define BUFFER_WAIT_SYNC    0
#define BUFFER_FILLING      1
#define BUFFER_FULL         2

Sampler::Sampler(uint8_t SignalPin)
: signalPin(SignalPin), isrCounter(0), sampleLine(0), bufferState(BUFFER_WAIT_SYNC)
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
  digitalWrite(13, sig);

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

