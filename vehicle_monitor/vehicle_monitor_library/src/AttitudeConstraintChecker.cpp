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

namespace VehicleMonitorLibrary{


AttitudeConstraintChecker::AttitudeConstraintChecker(
    double max_roll,
    double max_pitch,
    unsigned int projection_window,
    unsigned int motion_capture_system_frequency,
    std::shared_ptr<BaseVelocityEstimator> velocityEstimator)
:BaseConstraintChecker("ATTITUDE_CONSTRAINT_CHECKER"),
 projection_window_(projection_window),
 motion_capture_system_frequency_(motion_capture_system_frequency),
 velocity_estimator_(velocityEstimator),
 max_roll_(max_roll),
 max_pitch_(max_pitch) {

  if(velocity_estimator_ == nullptr){
    throw std::invalid_argument("Velocity Estimator cannot be nullptr");
  }
}

AttitudeConstraintChecker::~AttitudeConstraintChecker() {
}

bool AttitudeConstraintChecker::DoRegisterVehicle(std::shared_ptr<Vehicle> vehiclePtr) {

  if(velocity_estimator_->RegisterVehicle(vehiclePtr->GetID())) {
    return true;
  }

  return false;
}

bool AttitudeConstraintChecker::DoUnregisterVehicle(std::shared_ptr<Vehicle> vehiclePtr) {

  if(velocity_estimator_->UnregisterVehicle(vehiclePtr->GetID())) {
    return true;
  }

  return false;
}


void AttitudeConstraintChecker::DoCheckConstraint(const MotionCaptureSystemFrame& motionCaptureSystemFrame,
                                                  bool emergencyButtonPressed, std::map<std::string, bool>& checkResult) const {

  bool vehicleResult;

  velocity_estimator_->Update(motionCaptureSystemFrame);

  VehicleState frameElement;
  VehicleState estimatedVelocityState;

  float deltaTime = projection_window_ / (float)motion_capture_system_frequency_;

  for(const auto vehicleMapElement : _vehiclesMap) {

    if( motionCaptureSystemFrame.GetFrameElementForVehicle(vehicleMapElement.first, frameElement) == false) {
      continue;
    }

    if(velocity_estimator_->PredictVelocity(vehicleMapElement.second->GetID(),
                                            estimatedVelocityState) == false) {
      continue;
    }

    //    Eigen::Vector3d futureAngle(
    //        frameElement._angular.x() + estimatedVelocityState._angular.x() * deltaTime,
    //        frameElement._angular.y() + estimatedVelocityState._angular.y() * deltaTime,
    //        frameElement._angular.z() + estimatedVelocityState._angular.z() * deltaTime
    //    );

    // TODO(burrimi): Switch to predicted attitude.
    vehicleResult =  max_roll_ > std::abs(frameElement._angular.x()) &&
        max_pitch_ > std::abs(frameElement._angular.y());

    // false stays for constraint not satisfied
    checkResult.insert(make_pair(vehicleMapElement.first, vehicleResult));

  }
}

}
