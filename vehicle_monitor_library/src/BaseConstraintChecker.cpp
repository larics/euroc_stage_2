/**
 *    @file BaseConstraintChecker.cpp
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

#include "vehicle_monitor_library/BaseConstraintChecker.hpp"

#include "vehicle_monitor_library/Vehicle.hpp"
#include "vehicle_monitor_library/MotionCaptureSystemFrame.hpp"

using namespace std;

namespace VehicleMonitorLibrary {

BaseConstraintChecker::BaseConstraintChecker(std::string constraint_id)
    : constraint_id_(constraint_id) {}

BaseConstraintChecker::~BaseConstraintChecker() {}

std::string BaseConstraintChecker::getId() const { return constraint_id_; }

bool BaseConstraintChecker::registerVehicle(const std::string& vehicle_id) {
  last_valid_state_map_.insert(make_pair(vehicle_id, VehicleState()));
  return doRegisterVehicle(vehicle_id);
}

bool BaseConstraintChecker::doRegisterVehicle(const std::string& vehicle_id) {
  return true;
}

bool BaseConstraintChecker::unregisterVehicle(const std::string& vehicle_id) {
  last_valid_state_map_.erase(vehicle_id);
  return doUnregisterVehicle(vehicle_id);
}

bool BaseConstraintChecker::doUnregisterVehicle(const std::string& vehicle_id) {
  return true;
}

void BaseConstraintChecker::setVehiclesMap(
    std::shared_ptr<std::map<std::string, Vehicle::Ptr> > vehicles_map) {
  // Register all vehicles in constraint.
  for (const std::pair<std::string, Vehicle::Ptr>& vehicle_map_element :
       *vehicles_map) {
    registerVehicle(vehicle_map_element.second->getID());
  }

  vehicles_map_ = vehicles_map;
}

void BaseConstraintChecker::checkConstraint(
    const MotionCaptureSystemFrame& motion_capture_system_frame,
    bool emergency_button_pressed,
    std::map<std::string, ConstraintCheckerOutput>& check_result) {
  if (vehicles_map_ == nullptr) {
    std::cout << "[vehicle_monitor_library]: ERROR, vehicles_map_ not set!";
    return;
  }

  check_result.clear();

  // do preparation

  map<string, bool> constraint_satisfied_map;

  doCheckConstraint(motion_capture_system_frame, emergency_button_pressed,
                    constraint_satisfied_map);

  VehicleState tmp_frame;

  for (const auto& vehicleResult : constraint_satisfied_map) {
    if (vehicleResult.second == true &&
        motion_capture_system_frame.getFrameElementForVehicle(
            vehicleResult.first, tmp_frame)) {
      // store the new pose
      last_valid_state_map_[vehicleResult.first] = tmp_frame;
    }

    check_result.insert(make_pair(
        vehicleResult.first,
        ConstraintCheckerOutput(vehicleResult.second,
                                last_valid_state_map_[vehicleResult.first])));
  }
}
}
