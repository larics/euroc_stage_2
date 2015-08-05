/*
 * Copyright 2015 Alex Millane, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <iostream>

#include <ros/ros.h>

#include "kill_switch_library/kill_switch.h"

// Standard C++ entry point
int main(int argc, char** argv)
{
  // Announce this program to the ROS master
  ros::init(argc, argv, "kill_switch_node");
  // Start the node resource managers (communication, time, etc)
  ros::NodeHandle nh("~");

  // Getting the port name from the parameter server
  std::string port_name;
  nh.param<std::string>("port_name", port_name, "/dev/ttyUSB1");
  int check_frequency_hz;
  nh.param<int>("check_frequency_hz", check_frequency_hz, 10);

  // Create the kill switch
  kill_switch_library::KillSwitch kill_switch(check_frequency_hz);
  kill_switch.connect(port_name);
  kill_switch.start();

  // Looping and waiting until switch pressed
  double test_freq_hz = 1.0;
  ros::Rate loop_rate(test_freq_hz);
  while(ros::ok() && !kill_switch.getKillStatus()) {
    ROS_INFO("Everything ok.");
    loop_rate.sleep();
  }
  ROS_INFO("Kill switch pressed.");

  // Stopping the kill switch
  kill_switch.stop();
  // Stop the node's resources
  ros::shutdown();
  // Exit tranquilly
  return 0;
}
