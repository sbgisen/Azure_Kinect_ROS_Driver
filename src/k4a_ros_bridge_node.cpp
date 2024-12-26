// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// System headers
//
#include <chrono>
#include <memory>
#include <sstream>

// Library headers
//
#include "rclcpp/rclcpp.hpp"
#include <k4a/k4a.h>

// Project headers
//
#include "azure_kinect_ros_driver/k4a_ros_device.h"

class K4ADriver
{
public:
  K4ADriver(const rclcpp::Node::SharedPtr& node) : node_(node), device_(std::make_shared<K4AROSDevice>(node_))
  {
    startKinect();
    watchdog_timer_ =
        node_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&K4ADriver::watchdogTimerCallback, this));
  }
  void watchdogTimerCallback(){
    if(!device_->isRunning()){
      watchdog_timer_->cancel();
      RCLCPP_ERROR(node_->get_logger(), "K4A is not running");
      restartKinect();
      watchdog_timer_->reset();
    }
  }
  k4a_result_t startKinect()
  {
    k4a_result_t result = device_->startCameras();

    if (result != K4A_RESULT_SUCCEEDED)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to start cameras");
      return result;
    }

    result = device_->startImu();
    if (result != K4A_RESULT_SUCCEEDED)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to start IMU");
      return result;
    }

    RCLCPP_INFO(node_->get_logger(), "K4A Started");
    return result;
  }
  void restartKinect(){
    device_.reset();
    device_ = std::make_shared<K4AROSDevice>(node_);
    bool succeed = false;
    while(!succeed){
      succeed = startKinect() == K4A_RESULT_SUCCEEDED;
      if (!succeed)
      {
        device_.reset();
        RCLCPP_WARN(node_->get_logger(), "Failed to restart K4A, retrying in 1 second");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        device_ = std::make_shared<K4AROSDevice>(node_);
      }
    }
  }
  void run()
  {
    auto result = startKinect();

    if (result == K4A_RESULT_SUCCEEDED)
    {
      rclcpp::spin(node_);

      RCLCPP_INFO(node_->get_logger(), "ROS Exit Started");
    }

    device_.reset();

    RCLCPP_INFO(node_->get_logger(), "ROS Exit");

    rclcpp::shutdown();

    RCLCPP_INFO(node_->get_logger(), "ROS Shutdown complete");

    RCLCPP_INFO(node_->get_logger(), "Finished ros bridge main");
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<K4AROSDevice> device_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("k4a_ros_driver");
  auto k4a_driver = std::make_shared<K4ADriver>(node);
  k4a_driver->run();

  return 0;
}