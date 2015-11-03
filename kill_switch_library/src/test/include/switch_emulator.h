/*
 * Copyright 2015 Alex Millane, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#ifndef SWITCH_EMULATOR_H
#define SWITCH_EMULATOR_H

#include <ros/ros.h>

#include "kill_switch_library/uart.h"

class SwitchEmulator {
 public:
  // Constructor
  SwitchEmulator(double loop_frequency_hz = 100);

  // Connects to the external hardware
  bool connect(const std::string& port, int baudrate = 57600);

  // Returns the file descriptor to the psuedo tty port
  int getPtyfd() { return uart_.getPtyfd(); }

  // Starts the switch thread
  bool start();
  // Stops the switch thread
  void stop();

  // Simulates depressing and undpressing the phyiscal switch
  void trigger();
  void untrigger();

 private:
  // Main loop which process incoming serial data.
  void switchLoop();

  // The state of the switch
  bool kill_state_;
  // The serial port used to connect with the switch
  Uart uart_;
  // Flag indicating that the switch is connected to tty
  bool connected_;
  // Is the switch checking loop started
  bool started_;
  // Thread which periodically checks the switch
  std::thread* switch_thread_;
  // A flag indicating checking loop should stop
  bool stop_;
  // The frequency of checking switch
  double loop_frequency_hz_;
};

#endif  // SWITCH_EMULATOR_H
