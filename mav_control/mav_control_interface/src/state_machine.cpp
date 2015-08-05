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

#include "state_machine.h"

namespace mav_control_interface {

namespace state_machine {

StateMachineDefinition::StateMachineDefinition(std::shared_ptr<PositionControllerInterface> controller,
                                               ros::Publisher& command_publisher)
    : verbose_(false),
      controller_(controller),
      command_publisher_(command_publisher)
{

}

bool StateMachineDefinition::GetVerbose() const
{
  return verbose_;
}

void StateMachineDefinition::SetVerbose(bool verbose)
{
  verbose_ = verbose;
}

void StateMachineDefinition::SetParameters(const Parameters& parameters)
{
  parameters_ = parameters;
}

void StateMachineDefinition::PublishAttitudeCommand (
    const mav_msgs::EigenRollPitchYawrateThrust& command) const
{
  mav_msgs::RollPitchYawrateThrustPtr msg(new mav_msgs::RollPitchYawrateThrust);

  msg->header.stamp = ros::Time::now();  // TODO(acmarkus): get from msg
  mav_msgs::msgRollPitchYawrateThrustFromEigen(command, msg.get());
  command_publisher_.publish(msg);
}

} // end namespace state_machine

} // namespace mav_control_interface
