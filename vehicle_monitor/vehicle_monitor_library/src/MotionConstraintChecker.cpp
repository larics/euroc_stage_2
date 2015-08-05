/**
 *    @file MotionConstraintChecker.cpp
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

#include "vehicle_monitor_library/MotionConstraintChecker.hpp"

#include "vehicle_monitor_library/Vehicle.hpp"
#include "vehicle_monitor_library/BaseVelocityEstimator.hpp"
#include "vehicle_monitor_library/MotionCaptureSystemFrame.hpp"

using namespace std;

namespace VehicleMonitorLibrary{



MotionConstraintChecker::MotionConstraintChecker(std::shared_ptr<BaseVelocityEstimator> velocityEstimator)
:BaseConstraintChecker("MOTION_CONSTRAINT_CHECKER"),
 _velocityEstimator(velocityEstimator){

  if(_velocityEstimator == nullptr){
    throw std::invalid_argument("Velocity Estimator cannot be nullptr");
  }

}

MotionConstraintChecker::~MotionConstraintChecker(){

}

bool MotionConstraintChecker::DoRegisterVehicle(std::shared_ptr<Vehicle> vehiclePtr){

  if(_velocityEstimator->RegisterVehicle(vehiclePtr->GetID())){

    return true;

  }

  return false;

}

bool MotionConstraintChecker::DoUnregisterVehicle(std::shared_ptr<Vehicle> vehiclePtr){

  if(_velocityEstimator->UnregisterVehicle(vehiclePtr->GetID())){

    return true;

  }

  return false;


}


void MotionConstraintChecker::DoCheckConstraint(const MotionCaptureSystemFrame& motionCaptureSystemFrame,
                                                bool emergencyButtonPressed, std::map<std::string, bool>& checkResult) const{


  bool vehicleResult;

  _velocityEstimator->Update(motionCaptureSystemFrame);

  VehicleState frameElement;
  VehicleState estimatedVelocityState;

  for(const auto vehicleMapElement : _vehiclesMap){

    // 2) Retrieve current position and estimate velocity

    if( motionCaptureSystemFrame.GetFrameElementForVehicle(vehicleMapElement.first, frameElement) == false){
      continue;
    }

    if(_velocityEstimator->PredictVelocity(vehicleMapElement.second->GetID(),
                                           estimatedVelocityState) == false){
      continue;
    }

    // TODO: check constraints and produce output
    // TODO: think a better way to run the estimator only once.

    //checkResult.insert(make_pair(vehicleMapElement.first, vehicleResult));



  }

}


}
