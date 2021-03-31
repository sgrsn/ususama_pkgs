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

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Geometry>
#include "mecanum_io/mecanum_serial.hpp"

class MecanumNode : public rclcpp::Node
{
  public:
  MecanumNode()
  : Node("mecanum")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "topic", 10, std::bind(&MecanumNode::topic_callback, this, std::placeholders::_1));

    mecanum_ = MecanumSerial("/dev/ttyACM0", 115200);
  }

  private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
  {
    RCLCPP_INFO(this->get_logger(), "I heard");
    mecanum_.setVelocity( cmd_vel->linear.x, cmd_vel->linear.y, cmd_vel->angular.z );
  }
  
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  MecanumSerial mecanum_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumNode>());
  rclcpp::shutdown();
  return 0;
}