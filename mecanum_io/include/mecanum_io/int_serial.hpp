/*******************************************************************************
Copyright (c) 2021, Hidaka Sato
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met: 
1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution. 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies, 
either expressed or implied, of the FreeBSD Project.
*******************************************************************************/
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>
#include "mecanum_io/serial_io.hpp"

#define HEAD_BYTE   0x7E
#define ESCAPE_BYTE 0x7D
#define ESCAPE_MASK 0x20

class IntSerial
{
  private:
  SerialIO serial_;

  public:
  IntSerial(){}
  IntSerial(const char * device, uint32_t baud_rate)
  {
    serial_ = SerialIO(device, baud_rate);
    if(!serial_.hasFileDescriptor())
    {
      printf("Error, check the device");
    }
  }

  void writeCommand(uint8_t reg, int data)
  {
    uint8_t data_bytes[12] = {};
    uint8_t index = 0;
    uint8_t checksum = 0;
    data_bytes[index++] = HEAD_BYTE;
    data_bytes[index++] = reg;
    for (uint8_t i = 0; i < 4; ++i)
    {
        uint8_t tmp = data >> (24 - i*8) & 0xFF;
        if ( (tmp == ESCAPE_BYTE) || (tmp == HEAD_BYTE) )
        {
          data_bytes[index++] = ESCAPE_BYTE;
          checksum += ESCAPE_BYTE;
          data_bytes[index++] = tmp ^ ESCAPE_MASK;
          checksum += tmp ^ ESCAPE_MASK;
        }
        else
        {
          data_bytes[index++] = tmp;
          checksum += tmp;
        }
    }
    data_bytes[index] = checksum;
    serial_.writePort(data_bytes, index+1);
  }

  void forceCommand(uint8_t reg, uint8_t *command, uint8_t length)
  {
    uint8_t send_bytes[length+2] = {};
    send_bytes[0] = 0x7E;
    send_bytes[1] = reg;
    for(int i = 0; i < length; i++)
    {
      send_bytes[i+2] = command[i];
    }
    serial_.writePort(send_bytes, length+1);
  }

  void close()
  {
    serial_.closePort();
  }

};