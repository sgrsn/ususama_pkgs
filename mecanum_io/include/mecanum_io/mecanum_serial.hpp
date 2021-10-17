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

#include <iostream>
#include <memory>
#include <Eigen/Geometry>
#include "mecanum_io/int_serial.hpp"


#define COMMAND_POSE_X 0x01
#define COMMAND_POSE_Y 0x02
#define COMMAND_POSE_THETA 0x03
#define COMMAND_VELOCITY_X 0x04
#define COMMAND_VELOCITY_Y 0x05
#define COMMAND_VELOCITY_THETA 0x06

#define CURRENT_POSE_X 0x01
#define CURRENT_POSE_Y 0x02
#define CURRENT_POSE_THETA 0x03
#define CURRENT_VELOCITY_X 0x04
#define CURRENT_VELOCITY_Y 0x05
#define CURRENT_VELOCITY_THETA 0x06


#define COMMAND_SLEEP_MICROSEC 1000

class MecanumSerial
{
  private:
  IntSerial serial_;
  //std::unique_ptr<IntSerial> serial_;
  int register_[128] = {};

  public:
  MecanumSerial(){}
  MecanumSerial(const char * device, uint32_t baud_rate)
  {
    serial_ = IntSerial(device, baud_rate, register_);
  }

  // velocity shoud be set unit of mm/sec
  // angular_vel 10E-3rad/sec
  void setVelocity(Eigen::Vector2f velocity, float angular_vel=0)
  {
    serial_.writeCommand( COMMAND_VELOCITY_X, (int)velocity.x() );
    usleep(COMMAND_SLEEP_MICROSEC);
    serial_.writeCommand( COMMAND_VELOCITY_Y, (int)velocity.y() );
    usleep(COMMAND_SLEEP_MICROSEC);
    serial_.writeCommand( COMMAND_VELOCITY_THETA, (int)angular_vel );
    usleep(COMMAND_SLEEP_MICROSEC);
  }

  void setVelocity(float vel_x, float vel_y, float angular_vel=0)
  {
    serial_.writeCommand( COMMAND_VELOCITY_X, (int)vel_x );
    usleep(COMMAND_SLEEP_MICROSEC);
    serial_.writeCommand( COMMAND_VELOCITY_Y, (int)vel_y );
    usleep(COMMAND_SLEEP_MICROSEC);
    serial_.writeCommand( COMMAND_VELOCITY_THETA, (int)angular_vel );
    usleep(COMMAND_SLEEP_MICROSEC);
  }

  void update()
  {
    serial_.getIntData();
    //printf("theta : %d\r\n", register_[CURRENT_POSE_THETA]);
    //printf("vel1 : %d\r\n", register_[0x11]);
    //printf("vel2 : %d\r\n", register_[0x12]);
  }

  double PositionX()
  {
    return (double)register_[CURRENT_POSE_X];
  }
  double PositionY()
  {
    return (double)register_[CURRENT_POSE_Y];
  }
  double PositionTheta()
  {
    return (double)register_[CURRENT_POSE_THETA];
  }
  double VelocityX()
  {
    return (double)register_[CURRENT_VELOCITY_X];
  }
  double VelocityY()
  {
    return (double)register_[CURRENT_VELOCITY_Y];
  }
  double VelocityTheta()
  {
    return (double)register_[CURRENT_VELOCITY_THETA];
  }

  void stopMotors()
  {
    Eigen::Vector2f vel(0, 0);
    setVelocity( vel, (float)0.0 );
  }

  void close()
  {
    serial_.close();
  }
};