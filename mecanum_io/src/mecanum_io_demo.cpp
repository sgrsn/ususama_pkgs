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
#include <Eigen/Geometry>
#include "mecanum_io/mecanum_serial.hpp"

int main()
{
  MecanumSerial mecanum("/dev/ttyACM0", 115200);
  
  Eigen::Vector2f vel(100, 0);
  mecanum.setVelocity( vel, (float)0 );
  usleep(1000000);
  mecanum.stopMotors();
  usleep(100000);

  for(int i= 0; i < 5000; i++)
  {
    mecanum.getVelocity();
    usleep(100);
  }

/*
  Eigen::Vector2f vel2(0, 100);
  mecanum.setVelocity( vel2, (float)0 );
  usleep(1000000);
  mecanum.stopMotors();
  usleep(100000);

  Eigen::Vector2f vel3(0, 0);
  mecanum.setVelocity( vel3, (float)500 );
  usleep(1000000);
  mecanum.stopMotors();
  usleep(100000);
*/

  mecanum.close();
  return 0;
}