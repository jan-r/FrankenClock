#ifndef DCF77DECODER_H
#define DCF77DECODER_H

#include <Arduino.h>

#define NO_BIT    9

class DCF77Decoder
{
public:
  DCF77Decoder();
  void reset();
  void nextBit(uint8_t value);

private:
  int8_t    index;
  uint32_t  bits[2];

  void dataReady();
  bool checkParity(unsigned long bitsToCheck);
  bool checkRcvdStream();
};


#endif

