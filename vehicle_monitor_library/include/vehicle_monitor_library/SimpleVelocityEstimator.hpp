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

namespace VehicleMonitorLibrary{

class MotionCaptureSystemFrame;

class SimpleVelocityEstimator : public BaseVelocityEstimator{

 public:

  SimpleVelocityEstimator(unsigned int motionCaptureSystemFrequency);
  virtual ~SimpleVelocityEstimator();

  virtual void Update(const MotionCaptureSystemFrame& motionCaptureSystemFrame);

  virtual bool PredictVelocity(std::string vehicleID, VehicleState& estimatedVehicleState);

  bool RegisterVehicle(std::string vehicleID);
  bool UnregisterVehicle(std::string vehicleID);

 private:

  void InitMaps();

  const unsigned int _motionCaptureSystemFrequency;
  uint64_t _lastReceivedFrameNumber;

  std::map<std::string, bool> _initializedMap;
  std::map<std::string, uint64_t> _currentStateFrameNumberMap;
  std::map<std::string, VehicleState> _currentStateMap;
  std::map<std::string, uint64_t> _previousStateFrameNumberMap;
  std::map<std::string, VehicleState> _previousStateMap;
  std::map<std::string, uint64_t> _lastPredictionFrameNumberMap;
  std::map<std::string, VehicleState> _lastEstimatedStateMap;


};

}

#endif /* VML__SIMPLE_VELOCITY_ESTIMATOR_H_ */
