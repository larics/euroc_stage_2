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

#ifndef MAV_CONTROL_INTERFACE_PARAMETERS_H_
#define MAV_CONTROL_INTERFACE_PARAMETERS_H_

#include <math.h>

#include <mav_control_interface/deadzone.h>

namespace mav_control_interface {

class Parameters {
 public:
  static constexpr double kDefaultStickDeadzone = 0.1;
  static constexpr double kDefaultRcTeleopMaxCarrotDistance = 1.0;
  static constexpr double kDefaultRcTeleopMaxVelocity = 1.0;
  static constexpr double kDefaultRcMaxRollPitchCommand = 45.0 / 180.0 * M_PI;
  static constexpr double kDefaultRcMaxYawRateCommand = 45.0 / 180.0 * M_PI;

  Parameters()
      : stick_deadzone_(kDefaultStickDeadzone),
        rc_teleop_max_carrot_distance_(kDefaultRcTeleopMaxCarrotDistance),
        rc_teleop_max_velocity_(kDefaultRcTeleopMaxVelocity),
        rc_max_roll_pitch_command_(kDefaultRcMaxRollPitchCommand),
        rc_max_yaw_rate_command_(kDefaultRcMaxYawRateCommand)
  {
  }

  Deadzone<double> stick_deadzone_;
  double rc_teleop_max_carrot_distance_;
  double rc_teleop_max_velocity_;
  double rc_max_roll_pitch_command_;
  double rc_max_yaw_rate_command_;
};

} // end namespace mav_control_interface

#endif /* MAV_CONTROL_INTERFACE_PARAMETERS_H_ */
