/*
 * Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MAV_INTERFACE_H_
#define MAV_INTERFACE_H_

#include <aci/aci.h>
#include <aci/timesync_base.h>
#include <aci/uart.h>
#include <ros/ros.h>

#include <ethzasl_mav_interface/attitude_command_subscriber_trinity.h>
#include <ethzasl_mav_interface/common_data_publisher_autopilot.h>
#include <ethzasl_mav_interface/common_data_publisher_trinity.h>

namespace ethzasl_mav_interface {

class LocalClockRos : public aci::LocalClockBase {
 public:
  virtual double now();
};

class MavInterface {
 public:
  ~MavInterface();
  bool start(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

 private:
  static constexpr int kDefaultBaudrate = 460800;
  static constexpr int kDefaultPacketRateFast = 100;
  static constexpr int kDefaultPacketRateMedium = 50;
  static constexpr int kDefaultPacketRateSlow = 10;
  static constexpr int kMaxPacketRate = 100; // =engine rate, which is currently hard-coded into aci.
  static constexpr double kWatchdogTimeout = 1.0;
  bool setupSerialPort(ros::NodeHandle& private_nh, aci::UartPtr& uart);
  void slowPacketCallback();
  void watchdogCallback(const ros::TimerEvent& e);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  aci::UartPtr uart_;
  aci::Subscription slow_packet_sub_;

  std::shared_ptr<autopilot::CommonDataPublisher> common_data_autopilot_;
  std::shared_ptr<trinity::CommonDataPublisher> common_data_trinity_;
  std::shared_ptr<trinity::AttitudeCommandHandler> attitude_command_subscriber_trinity_;

  ros::Timer watchdog_;
};



} /* namespace ethzasl_mav_interface */

#endif /* MAV_INTERFACE_H_ */
