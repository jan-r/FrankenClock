#include "Sampler.h"

#define BUFFER_WAIT_SYNC    0
#define BUFFER_FILLING      1
#define BUFFER_FULL         2

#define SAMPLE_SETPOINT     40
#define MAX_CORRECTION     250

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
  digitalWrite(13, sig);

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
      if (bitCounter < 2)
      {
        Decoder.nextBit(NO_BIT);
      }
      else if ((bitCounter >= 6 ) && (bitCounter <= 13))
      {
        Decoder.nextBit(0);
      }
      else if (bitCounter > 15)
      {
        Decoder.nextBit(1);
      }
      else
      {
        Decoder.reset();
        Serial.print(bitCounter);
        Serial.write('\n');
      }
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

