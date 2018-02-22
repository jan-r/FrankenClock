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

#include "StreamReader.h"

StreamReader::StreamReader(const char *filename)
 : bitsLeft(0)
{
  fh = fopen(filename, "r");
  if (NULL == fh)
  {
    throw std::runtime_error("error opening file");
  }
}

int StreamReader::nextNibbleFromFile()
{
  int c = fgetc(fh);
  int value = -1;

  if (c != EOF)
  {
    // convert hexadecimal character back to integer value (0-15)
    if ((c >= '0') && (c <= '9'))
    {
      value = c - '0';
    }
    else if ((c >= 'A') && (c <= 'F'))
    {
      value = 10 + c - 'A';
    }
    else if ((c >= 'a') && (c <= 'f'))
    {
      value = 10 + c - 'a';
    }
  }
  return value;
}

int StreamReader::getNextSample()
{
  int ret;

  if (bitsLeft == 0)
  {
    bufferedNibble = nextNibbleFromFile();
    if (bufferedNibble < 0)
    {
       return -1;
    }
    bitsLeft = 4;
  }
  if (bufferedNibble & (1 << 3))
  {
    ret = 1;
  }
  else
  {
    ret = 0;
  }
  bitsLeft--;
  bufferedNibble <<= 1;
  return ret;
}

