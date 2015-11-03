/**
 *    @file VehicleMonitorLibrary/SimpleVelocityEstimator.hpp
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

#ifndef VML__SIMPLE_VELOCITY_ESTIMATOR_H_
#define VML__SIMPLE_VELOCITY_ESTIMATOR_H_

#include "BaseVelocityEstimator.hpp"

#include <stdint.h>

#include <map>

#include "VehicleState.hpp"

namespace VehicleMonitorLibrary {

class MotionCaptureSystemFrame;

class SimpleVelocityEstimator : public BaseVelocityEstimator {
 public:
  SimpleVelocityEstimator(unsigned int motionCaptureSystemFrequency);
  virtual ~SimpleVelocityEstimator();

  virtual void update(const std::string& vehicleID,
                      const MotionCaptureSystemFrame& motionCaptureSystemFrame);

  virtual bool predictVelocity(VehicleState& estimatedVehicleState);

 private:
  const unsigned int motion_capture_system_frequency_;
  uint64_t last_received_frame_number_;

  bool initialized_;
  uint64_t current_measurement_frame_number_;
  VehicleState current_measurement_;
  uint64_t previous_measurement_frame_number_;
  VehicleState previous_measurement_;
  uint64_t last_prediction_frame_number_;
  VehicleState last_estimated_state_;
};
}

#endif /* VML__SIMPLE_VELOCITY_ESTIMATOR_H_ */
