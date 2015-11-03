/**
 *    @file SimpleVelocityEstimator.cpp
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
 *    Implemented: Luca Gherardi
 *
 *    @author Luca Gherardi
 */

#include "vehicle_monitor_library/SimpleVelocityEstimator.hpp"

#include <string>
#include "vehicle_monitor_library/MotionCaptureSystemFrame.hpp"

namespace VehicleMonitorLibrary {

SimpleVelocityEstimator::SimpleVelocityEstimator(
    unsigned int motionCaptureSystemFrequency)
    : BaseVelocityEstimator(),
      last_received_frame_number_(0),
      motion_capture_system_frequency_(motionCaptureSystemFrequency) {
  initialized_ = false;
  current_measurement_frame_number_ = 0;
  current_measurement_ = VehicleState();
  previous_measurement_frame_number_ = 0;
  previous_measurement_ = VehicleState();
  last_prediction_frame_number_ = 0;
  last_estimated_state_ = VehicleState();
}

SimpleVelocityEstimator::~SimpleVelocityEstimator() {}

void SimpleVelocityEstimator::update(
    const std::string& vehicleID,
    const MotionCaptureSystemFrame& motionCaptureSystemFrame) {
  if (last_received_frame_number_ ==
      motionCaptureSystemFrame.getFrameNumber()) {
    // This is to avoid that two plugins that use the same
    // estimator call the update twice with the same data
    // If the first frame has id 0 we skip it, it's not
    // a big problem.
    return;
  }

  VehicleState tmpFrameElement;

  if (motionCaptureSystemFrame.getFrameElementForVehicle(vehicleID,
                                                         tmpFrameElement)) {
    if (initialized_ == false) {
      // we initialize the states with previous == current
      current_measurement_frame_number_ =
          motionCaptureSystemFrame.getFrameNumber();
      current_measurement_ = tmpFrameElement;
      previous_measurement_frame_number_ =
          motionCaptureSystemFrame.getFrameNumber();
      previous_measurement_ = tmpFrameElement;
      initialized_ = true;
    } else {
      previous_measurement_ = current_measurement_;
      previous_measurement_frame_number_ = current_measurement_frame_number_;

      current_measurement_ = tmpFrameElement;
      current_measurement_frame_number_ =
          motionCaptureSystemFrame.getFrameNumber();
    }
  }
}

bool SimpleVelocityEstimator::predictVelocity(
    VehicleState& estimatedVehicleState) {
  if (last_prediction_frame_number_ == current_measurement_frame_number_) {
    estimatedVehicleState = last_estimated_state_;

    return true;
  }

  if (previous_measurement_frame_number_ == current_measurement_frame_number_) {
    last_estimated_state_.velocity.x() = estimatedVehicleState.velocity.x() =
        0.0;
    last_estimated_state_.velocity.y() = estimatedVehicleState.velocity.y() =
        0.0;
    last_estimated_state_.velocity.z() = estimatedVehicleState.velocity.z() =
        0.0;
    last_estimated_state_.angular_rate.x() =
        estimatedVehicleState.angular_rate.x() = 0.0;
    last_estimated_state_.angular_rate.y() =
        estimatedVehicleState.angular_rate.y() = 0.0;
    last_estimated_state_.angular_rate.z() =
        estimatedVehicleState.angular_rate.z() = 0.0;

    return false;

  } else {
    // delta time in sec
    double deltaTime = (current_measurement_frame_number_ -
                        previous_measurement_frame_number_) /
                       (double)motion_capture_system_frequency_;

    estimatedVehicleState.velocity =
        current_measurement_.position - previous_measurement_.position;
    estimatedVehicleState.velocity /= deltaTime;

    // TODO(burrimi): detect wrap around on yaw!!!
    estimatedVehicleState.angular_rate =
        current_measurement_.orientation - previous_measurement_.orientation;
    estimatedVehicleState.angular_rate /= deltaTime;

    estimatedVehicleState.velocity_valid = true;

    last_estimated_state_ = estimatedVehicleState;

    return true;
  }
}
}
