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

#include <aci/simple_time_synchronizer.h>
#include <ethzasl_mav_interface/mav_interface.h>

namespace ethzasl_mav_interface {

constexpr int MavInterface::kDefaultBaudrate;
constexpr int MavInterface::kDefaultPacketRateFast;
constexpr int MavInterface::kDefaultPacketRateMedium;
constexpr int MavInterface::kDefaultPacketRateSlow;
constexpr int MavInterface::kMaxPacketRate;
constexpr double MavInterface::kWatchdogTimeout;

bool MavInterface::start(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {

  nh_ = nh;
  private_nh_ = private_nh;

  aci::Aci& aci = aci::instance();

  uart_.reset(new aci::Uart);

  bool connected = setupSerialPort(private_nh_, uart_);
  if (!connected) {
    ROS_ERROR("unable to connect");
    return false;
  }

  std::shared_ptr<aci::SimpleTimeSynchronizer> time_synchronizer(new aci::SimpleTimeSynchronizer);
  std::shared_ptr<LocalClockRos> ros_clock(new LocalClockRos);

  if (!aci.init(uart_, time_synchronizer, ros_clock)) {
    return false;
  }

  //TODO(acmarkus): reset all aci stuff.

  int packet_rate_fast, packet_rate_medium, packet_rate_slow;
  private_nh_.param("packet_rate_fast", packet_rate_fast, kDefaultPacketRateFast);
  private_nh_.param("packet_rate_medium", packet_rate_medium, kDefaultPacketRateMedium);
  private_nh_.param("packet_rate_slow", packet_rate_slow, kDefaultPacketRateSlow);

  if (packet_rate_fast < 0 || packet_rate_fast > kMaxPacketRate) {
    ROS_ERROR("packet_rate_fast has to be between 0 and %d, but is %d", kMaxPacketRate, packet_rate_fast);
    return false;
  }
  if (packet_rate_medium < 0 || packet_rate_medium > kMaxPacketRate) {
    ROS_ERROR("packet_rate_medium has to be between 0 and %d, but is %d", kMaxPacketRate, packet_rate_medium);
    return false;
  }
  if (packet_rate_slow < 0 || packet_rate_slow > kMaxPacketRate) {
    ROS_ERROR("packet_rate_slow has to be between 0 and %d, but is %d", kMaxPacketRate, packet_rate_slow);
    return false;
  }

  aci.setVariablePacketRate(aci::VAR_PACKET_FAST, packet_rate_fast);
  aci.setVariablePacketRate(aci::VAR_PACKET_MEDIUM, packet_rate_medium);
  aci.setVariablePacketRate(aci::VAR_PACKET_SLOW, packet_rate_slow);

  if (aci.getFcuType() == aci::FcuType::Trinity) {
    ROS_INFO("Starting components for trinity");
    common_data_trinity_.reset(new trinity::CommonDataPublisher(nh_, private_nh_));
    attitude_command_subscriber_trinity_.reset(new trinity::AttitudeCommandHandler(nh_, private_nh_));
    attitude_command_subscriber_trinity_->registerCommandedThrustCallback(
        std::bind(&trinity::CommonDataPublisher::setThrust, common_data_trinity_,
                  std::placeholders::_1));
  }
  else if (aci.getFcuType() == aci::FcuType::AutoPilot) {
    ROS_INFO("Starting components for autopilot");
    common_data_autopilot_.reset(new autopilot::CommonDataPublisher(nh_, private_nh_));
  }
  else {
    ROS_ERROR_STREAM("Could not determine fcu-type");
    return false;
  }

  slow_packet_sub_ = aci.registerVariableCallback(aci::VAR_PACKET_SLOW, &MavInterface::slowPacketCallback,
                                                  this);

  watchdog_ = nh.createTimer(ros::Duration(kWatchdogTimeout), &MavInterface::watchdogCallback, this, false,
                             true);

  return true;
}

MavInterface::~MavInterface() {
  aci::instance().shutdown();
}

bool MavInterface::setupSerialPort(ros::NodeHandle& private_nh, aci::UartPtr& uart) {
  bool connected = false;
  std::string port, portRX, portTX;
  int baudrate;

  private_nh.param("baudrate", baudrate, kDefaultBaudrate);

  if (private_nh.getParam("serial_port_rx", portRX) && private_nh.getParam("serial_port_tx", portTX)) {
    connected = uart->connect(portRX, portTX, baudrate);
  }
  else {
    private_nh.param("serial_port", port, std::string("/dev/ttyUSB0"));
    connected = uart->connect(port, port, baudrate);
  }
  return connected;
}

void MavInterface::slowPacketCallback() {
  watchdog_.stop();
  watchdog_.start();
}

void MavInterface::watchdogCallback(const ros::TimerEvent& e){
  if (aci::instance().isInitialized()) {
    ROS_FATAL("Watchdog timed out");
  }
  // TODO(acmarkus) what are we actually going to do? Shutdown? how about nodelets?
}

double LocalClockRos::now() {
  return ros::Time::now().toSec();
}

} /* namespace ethzasl_mav_interface */
