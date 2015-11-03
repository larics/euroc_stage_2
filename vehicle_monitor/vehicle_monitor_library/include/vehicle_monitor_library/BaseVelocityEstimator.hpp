/**
 *    @file VehicleMonitorLibrary/BaseVelocityEstimator.hpp
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

#ifndef VML__BASE_VELOCITY_ESTIMATOR_H_
#define VML__BASE_VELOCITY_ESTIMATOR_H_

#include "VehicleState.hpp"

#include <set>
#include <string>
#include <memory>

namespace VehicleMonitorLibrary {

class MotionCaptureSystemFrame;

class BaseVelocityEstimator {
 public:
  typedef std::shared_ptr<BaseVelocityEstimator> Ptr;

  BaseVelocityEstimator();
  virtual ~BaseVelocityEstimator();

  virtual void update(
      const std::string& vehicleID,
      const MotionCaptureSystemFrame& motionCaptureSystemFrame) = 0;

  virtual bool predictVelocity(VehicleState& estimatedVehicleState) = 0;

 protected:
};
}

#endif /* VML__BASE_VELOCITY_ESTIMATOR_H_ */
