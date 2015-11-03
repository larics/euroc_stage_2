/**
 *    @file Vehicle.cpp
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

#include "vehicle_monitor_library/Vehicle.hpp"

#include "vehicle_monitor_library/SimpleVelocityEstimator.hpp"
#include <sstream>

using namespace std;

namespace VehicleMonitorLibrary {

Vehicle::Vehicle(const std::string& id, double bounding_sphere_radius,
                 BaseVelocityEstimator::Ptr velocity_estimator)
    : id_(id),
      bounding_sphere_radius_(bounding_sphere_radius),
      velocity_estimator_(velocity_estimator),
      valid_velocity_(false),
      has_new_state_(false) {}

Vehicle::~Vehicle() {}

std::string Vehicle::getID() const { return id_; }

double Vehicle::getBoundingSphereRadius() const {
  return bounding_sphere_radius_;
}

void Vehicle::updateState(
    const MotionCaptureSystemFrame& motion_capture_system_frame) {
  VehicleState frame_element;
  if (motion_capture_system_frame.getFrameElementForVehicle(
          id_, frame_element) == false) {
    has_new_state_ = false;
    return;
  }

  if (frame_element.velocity_valid) {
    valid_velocity_ = true;
    has_new_state_ = true;
  } else {
    velocity_estimator_->update(id_, motion_capture_system_frame);

    VehicleState estimated_velocity_state;
    valid_velocity_ =
        velocity_estimator_->predictVelocity(estimated_velocity_state);
    if (valid_velocity_) {
      frame_element.copyVelocityAndRate(estimated_velocity_state);
      has_new_state_ = true;
    }
  }

  estimated_state_ = frame_element;
}

bool Vehicle::getState(VehicleState* estimated_state) {
  if (valid_velocity_) {
    *estimated_state = estimated_state_;
    return true;
  }
  return false;
}

std::string Vehicle::toString() const {
  stringstream ss;
  ss << "Vehicle:" << endl
     << "\tID = " << id_ << endl
     << "\tBounding Sphere Radius = " << bounding_sphere_radius_ << endl;
  return ss.str();
}
}

ostream& operator<<(ostream& outStream,
                    const VehicleMonitorLibrary::Vehicle& vehicle) {
  outStream << vehicle.toString();
  return outStream;
}
