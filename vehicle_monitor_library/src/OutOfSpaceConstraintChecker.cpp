/**
 *    @file OutOfSpaceConstraintChecker.cpp
 *
 *    Copyright 2014 Institute for Dynamic Systems and Control, ETH Zurich
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
 *    Designed:    Luca Gherardi
 *    Implemented: Caspar Reist and Luca Gherardi
 *
 *    @author Luca Gherardi
 */

#include "vehicle_monitor_library/OutOfSpaceConstraintChecker.hpp"
#include "vehicle_monitor_library/Vehicle.hpp"

using namespace std;

namespace VehicleMonitorLibrary {

OutOfSpaceConstraintChecker::OutOfSpaceConstraintChecker(
    BoundingVolume environmentBoundingVolume)
    : BaseConstraintChecker("OUT_OF_SPACE_CONSTRAINT_CHECKER"),
      _environmentBoundingVolume(environmentBoundingVolume) {}

OutOfSpaceConstraintChecker::~OutOfSpaceConstraintChecker() {}

void OutOfSpaceConstraintChecker::doCheckConstraint(
    const MotionCaptureSystemFrame& motion_capture_system_frame,
    bool emergency_button_pressed, std::map<std::string, bool>& check_result) {
  bool constraint_ok;
  VehicleState estimated_state;

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

    constraint_ok = _environmentBoundingVolume.isSphereInsideBB(
        estimated_state.position, vehicle->getBoundingSphereRadius());

    check_result.insert(make_pair(vehicle_id, constraint_ok));
  }
}
}
