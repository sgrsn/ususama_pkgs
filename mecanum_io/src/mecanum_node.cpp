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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Geometry>
#include "mecanum_io/mecanum_serial.hpp"
#include <string>

#define _USE_MATH_DEFINES
#include <cmath>
#include <chrono>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define RASPIMOUSE_EXPORT __attribute__ ((dllexport))
    #define RASPIMOUSE_IMPORT __attribute__ ((dllimport))
  #else
    #define RASPIMOUSE_EXPORT __declspec(dllexport)
    #define RASPIMOUSE_IMPORT __declspec(dllimport)
  #endif
  #ifdef RASPIMOUSE_BUILDING_DLL
    #define RASPIMOUSE_PUBLIC RASPIMOUSE_EXPORT
  #else
    #define RASPIMOUSE_PUBLIC RASPIMOUSE_IMPORT
  #endif
  #define RASPIMOUSE_PUBLIC_TYPE RASPIMOUSE_PUBLIC
  #define RASPIMOUSE_LOCAL
#else
  #define RASPIMOUSE_EXPORT __attribute__ ((visibility("default")))
  #define RASPIMOUSE_IMPORT
  #if __GNUC__ >= 4
    #define RASPIMOUSE_PUBLIC __attribute__ ((visibility("default")))
    #define RASPIMOUSE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define RASPIMOUSE_PUBLIC
    #define RASPIMOUSE_LOCAL
  #endif
  #define RASPIMOUSE_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif


using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MecanumNode : public rclcpp_lifecycle::LifecycleNode
{
  public:
  RASPIMOUSE_PUBLIC
  explicit MecanumNode(const rclcpp::NodeOptions & options)
  : //Node("mecanum"), 
    rclcpp_lifecycle::LifecycleNode("mecanum", options),
    ros_clock_(RCL_ROS_TIME), 
    last_odom_time_(0), 
    odom_transform_(rosidl_runtime_cpp::MessageInitialization::ZERO), 
    close_mecanum_(false)
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&MecanumNode::topic_callback, this, std::placeholders::_1));
    //mecanum_ = MecanumSerial("/dev/ttyACM0", 115200);
    //mecanum_ = std::make_unique<MecanumSerial>(port, 115200);
  }

  ~MecanumNode()
  {
    close_mecanum_ = true;
  }

  private:
  //rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  //on_configure(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &){return CallbackReturn::SUCCESS;}
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &){return CallbackReturn::SUCCESS;}
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &){return CallbackReturn::SUCCESS;}
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &){return CallbackReturn::SUCCESS;}

  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    std::string port;
    declare_parameter<std::string>("port", "/dev/ttyACM2");
    get_parameter("port", port);
    std::cout << port << "\n";
    mecanum_ = std::make_unique<MecanumSerial>(port.c_str(), 115200); 

    odom_timer_ = create_wall_timer(100ms, std::bind(&MecanumNode::publish_odometry, this));
    odom_transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
    odom_pub_->on_activate(); 

    // Publisher for odometry data
    //odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    odom_.header.frame_id = "odom";
    odom_.child_frame_id = "base_footprint";
    odom_.pose.pose.position.x = 0;
    odom_.pose.pose.position.y = 0;
    odom_.pose.pose.orientation.x = 0;
    odom_.pose.pose.orientation.y = 0;
    odom_.pose.pose.orientation.z = 0;
    odom_.pose.pose.orientation.w = 0;
    //odom_theta_ = 0;
    // Publisher for odometry transform
    odom_transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(
      this->shared_from_this());
    odom_transform_.header.frame_id = "odom";
    odom_transform_.child_frame_id = "base_footprint";
    odom_transform_.transform.translation.x = 0;
    odom_transform_.transform.translation.y = 0;
    odom_transform_.transform.rotation.x = 0;
    odom_transform_.transform.rotation.y = 0;
    odom_transform_.transform.rotation.z = 0;
    odom_transform_.transform.rotation.w = 0;
    // Timer for providing the odometry data
    //odom_timer_ = create_wall_timer(100ms, std::bind(&Raspimouse::publish_odometry, this));
    // Don't actually start publishing odometry data until activated
    //odom_timer_->cancel();
    read_thread_ = std::thread(std::bind(&MecanumNode::readThread, this));

    return CallbackReturn::SUCCESS;
  }

  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
  {
    //RCLCPP_INFO(this->get_logger(), "I heard");
    mecanum_->setVelocity( cmd_vel->linear.x*1000, cmd_vel->linear.y*1000, cmd_vel->angular.z*1000);
    linear_velocity_ = cmd_vel->linear.x;
    angular_velocity_ = cmd_vel->angular.z;
  }

  void readThread()
  {
    while(!close_mecanum_)
    {
      mecanum_->update();
    }
  }

  void publish_odometry()
  {
    //double laser_l = 0.165;
    //double odom_theta_ = mecanum_->PositionTheta() / 1000;
    //odom_.pose.pose.position.x = - (mecanum_->PositionX() / 1000 + laser_l*cosf(odom_theta_));
    //odom_.pose.pose.position.y = - (mecanum_->PositionY() / 1000 + laser_l*sinf(odom_theta_));
    double odom_theta_ = -mecanum_->PositionTheta() / 1000;
    odom_.pose.pose.position.x = mecanum_->PositionX() / 1000;
    odom_.pose.pose.position.y = mecanum_->PositionY() / 1000;
    last_odom_time_ = now();

    tf2::Quaternion odom_q;
    odom_q.setRPY(0, 0, odom_theta_);
    odom_.pose.pose.orientation.x = odom_q.x();
    odom_.pose.pose.orientation.y = odom_q.y();
    odom_.pose.pose.orientation.z = odom_q.z();
    odom_.pose.pose.orientation.w = odom_q.w();
    odom_.twist.twist.linear.x = linear_velocity_;
    odom_.twist.twist.angular.z = angular_velocity_;
    odom_.header.stamp = ros_clock_.now();
    odom_pub_->publish(odom_);

    odom_transform_.header.stamp = last_odom_time_;

    odom_transform_.transform.translation.x = odom_.pose.pose.position.x;
    odom_transform_.transform.translation.y = odom_.pose.pose.position.y;
    odom_transform_.transform.rotation.x = odom_q.x();
    odom_transform_.transform.rotation.y = odom_q.y();
    odom_transform_.transform.rotation.z = odom_q.z();
    odom_transform_.transform.rotation.w = odom_q.w();
    odom_transform_broadcaster_->sendTransform(odom_transform_);
  }
  
  rclcpp::Clock ros_clock_;
  rclcpp::Time last_odom_time_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry> > odom_pub_;
  std::thread read_thread_;
  //rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  //MecanumSerial mecanum_;
  std::unique_ptr<MecanumSerial> mecanum_;
  bool close_mecanum_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  nav_msgs::msg::Odometry odom_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_transform_broadcaster_;
  geometry_msgs::msg::TransformStamped odom_transform_;
  double linear_velocity_{0};
  double angular_velocity_{0};
};
/*
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumNode>());
  rclcpp::shutdown();
  return 0;
}*/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  rclcpp::NodeOptions options;
  std::shared_ptr<MecanumNode> mecanum_node = std::make_shared<MecanumNode>(options);
  exe.add_node(mecanum_node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
}