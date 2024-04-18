// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// Associated header
//
#include "azure_kinect_ros_driver/k4a_ros_bridge_nodelet.h"

// System headers
//

// Library headers
//
#include <pluginlib/class_list_macros.h>

// Project headers
//

PLUGINLIB_EXPORT_CLASS(Azure_Kinect_ROS_Driver::K4AROSBridgeNodelet, nodelet::Nodelet)

namespace Azure_Kinect_ROS_Driver
{
K4AROSBridgeNodelet::K4AROSBridgeNodelet() : Nodelet(), k4a_device(nullptr)
{
}

K4AROSBridgeNodelet::~K4AROSBridgeNodelet()
{
  this->k4a_device.reset(nullptr);
}

void K4AROSBridgeNodelet::watchdogTimerCallback(const ros::TimerEvent&)
{
  if (!k4a_device->isRunning() || !ros::ok() || ros::isShuttingDown())
  {
    timer_.stop();
    NODELET_ERROR("K4A device is not running. Restarting device.");
    restartKinect();
    timer_.start();
  }
}
void K4AROSBridgeNodelet::startKinect()
{
  k4a_device = std::unique_ptr<K4AROSDevice>(new K4AROSDevice(getNodeHandle(), getPrivateNodeHandle()));
  if (k4a_device->startCameras() != K4A_RESULT_SUCCEEDED)
  {
    k4a_device.reset(nullptr);
    throw nodelet::Exception("Could not start K4A cameras");
  }
  NODELET_INFO("Cameras started");

  if (k4a_device->startImu() != K4A_RESULT_SUCCEEDED)
  {
    k4a_device.reset(nullptr);
    throw nodelet::Exception("Could not start K4A IMU");
  }
  NODELET_INFO("IMU started");
}

void K4AROSBridgeNodelet::restartKinect()
{
  this->k4a_device.reset(nullptr);
  k4a_device = std::unique_ptr<K4AROSDevice>(new K4AROSDevice(getNodeHandle(), getPrivateNodeHandle()));
  bool succeed = false;
  while (!succeed)
  {
    if (k4a_device->startCameras() != K4A_RESULT_SUCCEEDED)
    {
      k4a_device.reset(nullptr);
      NODELET_WARN("Could not start K4A cameras");
      sleep(1.0);
      k4a_device = std::unique_ptr<K4AROSDevice>(new K4AROSDevice(getNodeHandle(), getPrivateNodeHandle()));
      continue;
    }
    NODELET_INFO("Cameras started");

    if (k4a_device->startImu() != K4A_RESULT_SUCCEEDED)
    {
      k4a_device.reset(nullptr);
      NODELET_WARN("Could not start K4A IMU");
      sleep(1.0);
      k4a_device = std::unique_ptr<K4AROSDevice>(new K4AROSDevice(getNodeHandle(), getPrivateNodeHandle()));
      continue;
    }
    NODELET_INFO("IMU started");
    succeed = true;
    break;
  }
}

void K4AROSBridgeNodelet::onInit()
{
  NODELET_INFO("K4A ROS Nodelet Start");
  startKinect();
  timer_ = getNodeHandle().createTimer(ros::Duration(0.1), &K4AROSBridgeNodelet::watchdogTimerCallback, this);
  NODELET_INFO("Watchdog timer started");
}
}  // namespace Azure_Kinect_ROS_Driver
