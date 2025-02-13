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

class K4ADriver : public rclcpp::Node
{
public:
  K4ADriver(const rclcpp::NodeOptions &options)
      : rclcpp::Node("k4a_ros_driver", options) {
    watchdog_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&K4ADriver::watchdogTimerCallback, this));
  }
  void watchdogTimerCallback(){
    if (!device_) {
      startKinect();
    }
    if(!device_->isRunning()){
      watchdog_timer_->cancel();
      RCLCPP_ERROR(this->get_logger(), "K4A is not running");
      restartKinect();
      watchdog_timer_->reset();
    }
  }
  k4a_result_t startKinect()
  {
    if (!device_)
    {
      device_ = std::make_shared<K4AROSDevice>(shared_from_this());
    }

    k4a_result_t result = device_->startCameras();

    if (result != K4A_RESULT_SUCCEEDED)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to start cameras");
      return result;
    }

    result = device_->startImu();
    if (result != K4A_RESULT_SUCCEEDED)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to start IMU");
      return result;
    }

    RCLCPP_INFO(this->get_logger(), "K4A Started");
    return result;
  }
  void restartKinect(){
    device_.reset();
    device_ = std::make_shared<K4AROSDevice>(shared_from_this());
    bool succeed = false;
    while(!succeed){
      succeed = startKinect() == K4A_RESULT_SUCCEEDED;
      if (!succeed)
      {
        device_.reset();
        RCLCPP_WARN(this->get_logger(), "Failed to restart K4A, retrying in 1 second");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        device_ = std::make_shared<K4AROSDevice>(shared_from_this());
      }
    }
  }
  void run()
  {
    rclcpp::spin(shared_from_this());

    RCLCPP_INFO(this->get_logger(), "ROS Exit Started");

    device_.reset();

    RCLCPP_INFO(this->get_logger(), "ROS Exit");

    rclcpp::shutdown();

    RCLCPP_INFO(this->get_logger(), "ROS Shutdown complete");

    RCLCPP_INFO(this->get_logger(), "Finished ros bridge main");
  }

private:
  std::shared_ptr<K4AROSDevice> device_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto k4a_driver = std::make_shared<K4ADriver>(rclcpp::NodeOptions());
  k4a_driver->run();

  return 0;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(K4ADriver)
