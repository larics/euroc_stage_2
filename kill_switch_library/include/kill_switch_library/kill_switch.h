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

#ifndef KILL_SWITCH_H
#define KILL_SWITCH_H

#include <thread>
#include <iostream>

#include <ros/ros.h>

#include "kill_switch_library/uart.h"

namespace kill_switch_library {

class KillSwitch
{

 public:

  // Constructor
  KillSwitch(double check_frequency_hz = 10);
  ~KillSwitch();

  // Connects to the external hardware
  bool connect(const std::string& port, int baudrate = 57600);
  // Returns the status of the switch
  bool getKillStatus() const { return kill_status_; }
  // Starts the checking thread
  bool start();
  // Stops the checking thread
  void stop();
  // Resets the kill status
  bool reset();

 private:

  // Check loop
  void checkLoop();
  // Checks the switch
  bool check();

  // The serial port used to connect with the switch
  Uart uart_;
  // The current char sent to check switch status
  char check_char_;
  // The status of the switch
  bool kill_status_;
  // The rate of switch checking
  double check_frequency_hz_;
  // The check char receive timeout
  double timeout_ms_;
  // The handle to the switch checking thread
  std::thread* checkingThread_;
  // Flag indicating that checker thread should terminate
  bool stop_;
  // A flag indicating if the checking loop has started or not
  bool started_;
  // A flag indicating if the switch has been connected to the hardware or not
  bool connected_;

};

}

#endif // KILL_SWITCH_H
