/**
 *    @file VehicleMonitorLibrary/AttitudeConstraintChecker.hpp
 *
 *    Copyright 2015 Autonomous Systems Lab, ETH Zurich
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 *
 *    Designed:    Michael Burri
 *    Implemented: Michael Burri
 *
 *    @author Michael Burri
 */

#ifndef VML__ATTITUDE_CONSTRAINT_CHECKER_H_
#define VML__ATTITUDE_CONSTRAINT_CHECKER_H_

#include <Eigen/Eigen>

#include "vehicle_monitor_library/BaseConstraintChecker.hpp"
#include "vehicle_monitor_library/BoundingVolume.hpp"

namespace octomap {
class OcTree;
}

namespace VehicleMonitorLibrary {

class BaseVelocityEstimator;

class AttitudeConstraintChecker : public BaseConstraintChecker {
 public:
  AttitudeConstraintChecker(double max_roll, double max_pitch,
                            unsigned int projection_window,
                            unsigned int motion_capture_system_frequency);

  virtual ~AttitudeConstraintChecker();

 protected:
  virtual void doCheckConstraint(
      const MotionCaptureSystemFrame& motion_capture_system_frame,
      bool emergency_button_pressed, std::map<std::string, bool>& check_result);

 private:
  // number of time steps used to compute the future position of a vehicle
  unsigned int projection_window_;

  unsigned int motion_capture_system_frequency_;

  double max_roll_;
  double max_pitch_;
};
}
#endif /* VML__ATTITUDE_CONSTRAINT_CHECKER_H_ */
