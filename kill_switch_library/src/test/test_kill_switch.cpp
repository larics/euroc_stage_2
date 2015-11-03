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

#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "kill_switch_library/kill_switch.h"
#include "switch_emulator.h"

TEST(kill_switch_library, test) {
  // Starting this test as a ros node
  int argc = 0;
  char** argv;
  ros::init(argc, argv, "kill_switch_node");
  ros::start();

  // Creating the switch emulator
  SwitchEmulator switch_emulator;
  switch_emulator.connect("/dev/ptmx");
  switch_emulator.start();
  int pt = switch_emulator.getPtyfd();

  // Getting the psuedo terminal slave
  grantpt(pt);
  unlockpt(pt);
  std::string port = ptsname(pt);

  // Creating the kill switch
  int check_rate = 10;
  kill_switch_library::KillSwitch kill_switch(check_rate);
  bool connect_result = kill_switch.connect(port);
  ASSERT_TRUE(connect_result) << "Unable to connect to psuedo tty";
  bool start_result = kill_switch.start();
  ASSERT_TRUE(start_result) << "Unable to start kill_switch";

  // Check startup state of the switch
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  ROS_INFO("Initial check");
  EXPECT_FALSE(kill_switch.getKillStatus())
      << "Initial: Switch triggered when it should not be";

  // Triggering the switch and check
  switch_emulator.trigger();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  ROS_INFO("Trigger check");
  EXPECT_TRUE(kill_switch.getKillStatus())
      << "Post-Trigger: Switch not triggered when it should be";

  // Untriggering and checking it still is killed
  switch_emulator.untrigger();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  ROS_INFO("Untrigger check");
  EXPECT_TRUE(kill_switch.getKillStatus())
      << "Post-Untrigger: Switch not triggered when it should be";

  // Resetting the checking kill status reset
  kill_switch.reset();
  kill_switch.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  ROS_INFO("Reset check");
  EXPECT_FALSE(kill_switch.getKillStatus())
      << "Post-reset: Switch triggered when it should not be";

  // Triggering the switch and check
  switch_emulator.trigger();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  ROS_INFO("Re-Trigger check");
  EXPECT_TRUE(kill_switch.getKillStatus())
      << "Post-Trigger: Switch not triggered when it should be";

  // Stopping the components
  kill_switch.stop();
  switch_emulator.stop();

  // Stop the node's resources
  ros::shutdown();
}

/*
 *	GTests Main
 */

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
