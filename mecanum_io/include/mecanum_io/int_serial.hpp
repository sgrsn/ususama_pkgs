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
#include <list>
#include "mecanum_io/serial_io.hpp"

#define HEAD_BYTE   0x1D
#define READ_COMMAND   0xFF
#define ESCAPE_BYTE 0x1E
#define ESCAPE_MASK 0x1F

class IntSerial
{
  private:
  SerialIO serial_;
  int *register_;

  public:
  IntSerial(){}
  IntSerial(const char * device, uint32_t baud_rate, int *register_p)
  {
    register_ = register_p;
    serial_ = SerialIO(device, baud_rate);
    if(!serial_.hasFileDescriptor())
    {
      printf("Error, check the device");
    }
  }

  void writeCommand(uint8_t reg, int data)
  {
    std::list<uint8_t> packet_list;
    packet_list.push_back(HEAD_BYTE);
    uint8_t checksum = 0;
    packet_list.push_back(reg);
    checksum += reg;

    uint8_t bytes[] = {
      (uint8_t)(data>>24 & 0xFF), 
      (uint8_t)(data>>16 & 0xFF), 
      (uint8_t)(data>>8 & 0xFF), 
      (uint8_t)(data>>0 & 0xFF), 
    };

    for(int i = 0; i < 4; i++)
    {
      if( (bytes[i] == ESCAPE_BYTE) || (bytes[i] == HEAD_BYTE) )
      {
        packet_list.push_back(ESCAPE_BYTE);
        checksum += ESCAPE_BYTE;
        uint8_t escape = (uint8_t)( (int)bytes[i] ^ (int)ESCAPE_MASK );
        packet_list.push_back(escape);
        checksum += escape;
      }
      else
      {
        packet_list.push_back(bytes[i]);
        checksum += bytes[i];
      }
    }
    packet_list.push_back(checksum);
    int size = packet_list.size() - 1;
    packet_list.insert(std::next(packet_list.begin(), 1), (uint8_t)size);
    uint8_t packet_array[packet_list.size()] = {};
    std::copy(packet_list.begin(), packet_list.end(), packet_array);
    serial_.writePort(packet_array, packet_list.size());
  }

  void getIntData()
  {
    uint8_t data_bytes[12] = {};
    serial_.readPort(data_bytes, 1);
    if (data_bytes[0] != HEAD_BYTE)
    {
      return;
    }
    serial_.readPort(data_bytes, 1);
    int size = data_bytes[0];
    if(size > 12) return;
    serial_.readPort(data_bytes, size);
    int index = 0;
    uint8_t reg = data_bytes[0];
    index++;
    int8_t checksum = 0;
    checksum += reg;
    uint8_t bytes[4] = {0,0,0,0};
    for (int i = 0; i < 4; ++i)
    {
      uint8_t d = data_bytes[index++];
      if (d == ESCAPE_BYTE)
      {
        uint8_t nextByte = data_bytes[index++];
        bytes[i] = nextByte ^ ESCAPE_MASK;
        checksum += (d + nextByte);
      }
      else
      {
          bytes[i] = d;
          checksum += d;
      }
    }
    int8_t checksum_recv = data_bytes[index++];
    int32_t DATA = 0x00;
    for(int i = 0; i < 4; i++)
    {
        DATA |= (((int32_t)bytes[i]) << (24 - (i*8)));
    }

    //printf("get data %d\r\n", DATA);

    if (checksum == checksum_recv)
    {
        register_[reg] =  DATA;
    }
    else
    {
        // data error
    }
    
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