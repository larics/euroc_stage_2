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

#include <mav_msgs/default_topics.h>
#include <std_msgs/String.h>

#include "state_machine.h"

namespace mav_control_interface {

namespace state_machine {

StateMachineDefinition::StateMachineDefinition(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
                                               std::shared_ptr<PositionControllerInterface> controller)
    : verbose_(false),
      controller_(controller)
{
  command_publisher_ = nh.advertise<mav_msgs::RollPitchYawrateThrust>(
      mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1);

  state_info_publisher_ = nh.advertise<std_msgs::String>("state_machine/state_info", 1, true);
}

void StateMachineDefinition::SetParameters(const Parameters& parameters)
{
  parameters_ = parameters;
}

void StateMachineDefinition::PublishAttitudeCommand (
    const mav_msgs::EigenRollPitchYawrateThrust& command) const
{
  mav_msgs::RollPitchYawrateThrustPtr msg(new mav_msgs::RollPitchYawrateThrust);

  mav_msgs::EigenRollPitchYawrateThrust tmp_command = command;
  tmp_command.thrust.x() = 0;
  tmp_command.thrust.y() = 0;
  tmp_command.thrust.z() = std::max(0.0, command.thrust.z());

  msg->header.stamp = ros::Time::now();  // TODO(acmarkus): get from msg
  mav_msgs::msgRollPitchYawrateThrustFromEigen(command, msg.get());
  command_publisher_.publish(msg);
}

void StateMachineDefinition::PublishStateInfo(const std::string& info)
{
  if (state_info_publisher_.getNumSubscribers() > 0) {
    std_msgs::StringPtr msg(new std_msgs::String);
    msg->data = info;
    state_info_publisher_.publish(msg);
  }
}

} // end namespace state_machine

} // namespace mav_control_interface
