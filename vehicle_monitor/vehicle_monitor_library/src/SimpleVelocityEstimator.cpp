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

namespace VehicleMonitorLibrary{

SimpleVelocityEstimator::SimpleVelocityEstimator(unsigned int motionCaptureSystemFrequency)
: BaseVelocityEstimator(),
  _lastReceivedFrameNumber(0),
  _motionCaptureSystemFrequency(motionCaptureSystemFrequency){

  //InitMaps();

}

SimpleVelocityEstimator::~SimpleVelocityEstimator(){

}

void SimpleVelocityEstimator::Update(
    const MotionCaptureSystemFrame& motionCaptureSystemFrame){

  if(_lastReceivedFrameNumber == motionCaptureSystemFrame.GetFrameNumber()){

    // This is to avoid that two plugins that use the same
    // estimator call the update twice with the same data
    // If the first frame has id 0 we skip it, it's not
    // a big problem.
    return;

  }

  VehicleState tmpFrameElement;

  for(auto ID : _vehicleIDs){

    if(motionCaptureSystemFrame.GetFrameElementForVehicle(
        ID, tmpFrameElement)){


      if(_initializedMap[ID] == false){

        // we initialize the states with previous == current

        _currentStateFrameNumberMap[ID] = motionCaptureSystemFrame.GetFrameNumber();
        _currentStateMap[ID] = tmpFrameElement;
        _previousStateFrameNumberMap[ID] = motionCaptureSystemFrame.GetFrameNumber();
        _previousStateMap[ID] = tmpFrameElement;
        _initializedMap[ID] = true;

      }else{

        _previousStateMap[ID] = std::move(_currentStateMap[ID]);
        _previousStateFrameNumberMap[ID] = std::move(_currentStateFrameNumberMap[ID]);

        _currentStateMap[ID] = tmpFrameElement;
        _currentStateFrameNumberMap[ID] = motionCaptureSystemFrame.GetFrameNumber();

      }

    }

  }

}

bool SimpleVelocityEstimator::PredictVelocity(std::string vehicleID,
                                              VehicleState& estimatedVehicleState){

  if(_lastPredictionFrameNumberMap[vehicleID] == _currentStateFrameNumberMap[vehicleID]){

    estimatedVehicleState = _lastEstimatedStateMap[vehicleID];

    return true;

  }

  if(_previousStateFrameNumberMap.at(vehicleID) == _currentStateFrameNumberMap.at(vehicleID)){

    _lastEstimatedStateMap[vehicleID]._linear.x() =
        estimatedVehicleState._linear.x() = 0.0;
    _lastEstimatedStateMap[vehicleID]._linear.y() =
        estimatedVehicleState._linear.y() = 0.0;
    _lastEstimatedStateMap[vehicleID]._linear.z() =
        estimatedVehicleState._linear.z() = 0.0;
    _lastEstimatedStateMap[vehicleID]._angular.x() =
        estimatedVehicleState._angular.x() = 0.0;
    _lastEstimatedStateMap[vehicleID]._angular.y() =
        estimatedVehicleState._angular.y() = 0.0;
    _lastEstimatedStateMap[vehicleID]._angular.z() =
        estimatedVehicleState._angular.z() = 0.0;

    return false;

  }else{

    // delta time in sec
    float deltaTime = (_currentStateFrameNumberMap.at(vehicleID) - _previousStateFrameNumberMap.at(vehicleID))
										    / (float)_motionCaptureSystemFrequency;

    estimatedVehicleState._linear = _currentStateMap[vehicleID]._linear - _previousStateMap[vehicleID]._linear;
    estimatedVehicleState._linear /= deltaTime;

    estimatedVehicleState._angular = _currentStateMap[vehicleID]._angular - _previousStateMap[vehicleID]._angular;
    estimatedVehicleState._angular /= deltaTime;

    _lastEstimatedStateMap[vehicleID] = estimatedVehicleState;

    return true;

  }

}

bool SimpleVelocityEstimator::RegisterVehicle(std::string vehicleID){

  if(BaseVelocityEstimator::RegisterVehicle(vehicleID)){

    _currentStateFrameNumberMap.insert(make_pair(vehicleID, 0));
    _currentStateMap.insert(make_pair(vehicleID, VehicleState()));
    _previousStateFrameNumberMap.insert(make_pair(vehicleID, 0));
    _previousStateMap.insert(make_pair(vehicleID, VehicleState()));
    _initializedMap.insert(make_pair(vehicleID, false));
    _lastPredictionFrameNumberMap.insert(make_pair(vehicleID, 0));
    _lastEstimatedStateMap.insert(make_pair(vehicleID, VehicleState()));

    std::cout << "[SimpleVelocityEstimator] Registered vehicle " << vehicleID << std::endl;

    return true;

  }

  return false;

}

bool SimpleVelocityEstimator::UnregisterVehicle(std::string vehicleID){

  if(BaseVelocityEstimator::UnregisterVehicle(vehicleID)){

    _currentStateFrameNumberMap.erase(vehicleID);
    _currentStateMap.erase(vehicleID);
    _previousStateFrameNumberMap.erase(vehicleID);
    _previousStateMap.erase(vehicleID);
    _initializedMap.erase(vehicleID);
    _lastPredictionFrameNumberMap.erase(vehicleID);
    _lastEstimatedStateMap.erase(vehicleID);

    std::cout << "[SimpleVelocityEstimator] Unregistered vehicle " << vehicleID << std::endl;

    return true;

  }

  return false;

}

void SimpleVelocityEstimator::InitMaps(){

  _currentStateFrameNumberMap.clear();
  _currentStateMap.clear();
  _previousStateFrameNumberMap.clear();
  _previousStateMap.clear();
  _initializedMap.clear();
  _lastPredictionFrameNumberMap.clear();
  _lastEstimatedStateMap.clear();

  for(auto ID : _vehicleIDs){

    _currentStateFrameNumberMap.insert(make_pair(ID, 0));
    _currentStateMap.insert(make_pair(ID, VehicleState()));
    _previousStateFrameNumberMap.insert(make_pair(ID, 0));
    _previousStateMap.insert(make_pair(ID, VehicleState()));
    _initializedMap.insert(make_pair(ID, false));
    _lastPredictionFrameNumberMap.insert(make_pair(ID, 0));
    _lastEstimatedStateMap.insert(make_pair(ID, VehicleState()));

  }

}



}
