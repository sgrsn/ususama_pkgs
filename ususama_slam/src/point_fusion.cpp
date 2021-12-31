#include <stdio.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "laser_geometry/laser_geometry.h"

#include "stdafx.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>

class PointFusion : public rclcpp::Node
{
public:
  PointFusion()
  : Node("point_fusion")
  {
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&PointFusion::callback, this, std::placeholders::_1));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    cloud1.width    = 726;
  }
private:
  void callback(sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    if(msg->header.frame_id == "laser_A")
    {
      RCLCPP_INFO(this->get_logger(), "I heard: %s", msg->header.frame_id.c_str());
      sensor_msgs::PointCloud2 cloud;
      projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tfListener_);
      point_cloud_publisher_.publish(cloud);
    }
    else if(msg->header.frame_id == "laser_B")
    {
      RCLCPP_INFO(this->get_logger(), "I heard: %s", msg->header.frame_id.c_str());
    }
  }
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  laser_geometry::LaserProjection projector_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

class My_Filter {
public:
  My_Filter()
  {
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &My_Filter::scanCallback, this);
    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
    tfListener_.setExtrapolationLimit(ros::Duration(0.1));
  }
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tfListener_);
    point_cloud_publisher_.publish(cloud);
  }
private:
  ros::NodeHandle node_;
  tf::TransformListener tfListener_;

  ros::Publisher point_cloud_publisher_;
  ros::Subscriber scan_sub_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointFusion>());
  rclcpp::shutdown();
  return 0;
}