/**
 *    @file AttitudeConstraintChecker.cpp
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

#include "vehicle_monitor_library/AttitudeConstraintChecker.hpp"
#include "vehicle_monitor_library/Vehicle.hpp"
#include "vehicle_monitor_library/VehicleMonitor.hpp"
#include "vehicle_monitor_library/BoundingVolume.hpp"
#include "vehicle_monitor_library/BaseVelocityEstimator.hpp"
#include "vehicle_monitor_library/MotionCaptureSystemFrame.hpp"

using namespace std;

namespace VehicleMonitorLibrary {

AttitudeConstraintChecker::AttitudeConstraintChecker(
    double max_roll, double max_pitch, unsigned int projection_window,
    unsigned int motion_capture_system_frequency)
    : BaseConstraintChecker("ATTITUDE_CONSTRAINT_CHECKER"),
      projection_window_(projection_window),
      motion_capture_system_frequency_(motion_capture_system_frequency),
      max_roll_(max_roll),
      max_pitch_(max_pitch) {}

AttitudeConstraintChecker::~AttitudeConstraintChecker() {}

void AttitudeConstraintChecker::doCheckConstraint(
    const MotionCaptureSystemFrame& motion_capture_system_frame,
    bool emergency_button_pressed, std::map<std::string, bool>& check_result) {
  bool constraint_ok;

  VehicleState estimated_state;

  double deltaTime =
      projection_window_ / (double)motion_capture_system_frequency_;

  for (const std::pair<std::string, Vehicle::Ptr>& vehicle_map_element :
       *vehicles_map_) {
    const std::string& vehicle_id = vehicle_map_element.first;
    Vehicle::Ptr vehicle = vehicle_map_element.second;
    if (vehicle->getHasNewState() == false) {
      continue;
    }

    if (vehicle->getState(&estimated_state) == false) {
      continue;
    }

    //    Eigen::Vector3d futureAngle(
    //        frameElement._angular.x() + estimatedVelocityState._angular.x() *
    //        deltaTime,
    //        frameElement._angular.y() + estimatedVelocityState._angular.y() *
    //        deltaTime,
    //        frameElement._angular.z() + estimatedVelocityState._angular.z() *
    //        deltaTime
    //    );

    // TODO(burrimi): Switch to predicted attitude.
    constraint_ok = max_roll_ > std::abs(estimated_state.orientation.x()) &&
                    max_pitch_ > std::abs(estimated_state.orientation.y());

    // false stays for constraint not satisfied
    check_result.insert(make_pair(vehicle_id, constraint_ok));
  }
}
}
