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

namespace VehicleMonitorLibrary{



BaseConstraintChecker::BaseConstraintChecker(std::string ID)
:_ID(ID){

}

BaseConstraintChecker::~BaseConstraintChecker(){

}

std::string BaseConstraintChecker::GetID() const{

  return _ID;

}

bool BaseConstraintChecker::RegisterVehicle(std::shared_ptr<Vehicle> vehiclePtr){

  pair<map<string, std::shared_ptr<Vehicle> >::iterator,bool> ret;

  ret = _vehiclesMap.insert(make_pair(vehiclePtr->GetID(), vehiclePtr));

  if (ret.second==false) {
    std::cout << "VEHICLE REGISTER FAILED FOR "  << _ID << std::endl;
    return false;

  }

  _lastValidStateMap.insert(make_pair(vehiclePtr->GetID(), VehicleState()));

  if(DoRegisterVehicle(vehiclePtr) == false){
    std::cout << "VEHICLE DO_REGISTER_VEHICLE FAILED FOR "  << _ID << std::endl;

    _vehiclesMap.erase(vehiclePtr->GetID());
    _lastValidStateMap.erase(vehiclePtr->GetID());

  }

  return true;

}

bool BaseConstraintChecker::DoRegisterVehicle(std::shared_ptr<Vehicle> vehiclePtr){

  return true;

}

bool BaseConstraintChecker::UnregisterVehicle(std::shared_ptr<Vehicle> vehiclePtr){

  map<string, std::shared_ptr<Vehicle> >::iterator mapElement;

  mapElement = _vehiclesMap.find(vehiclePtr->GetID());

  if(mapElement == _vehiclesMap.end()){
    return false;
  }

  _vehiclesMap.erase(mapElement);
  _lastValidStateMap.erase(vehiclePtr->GetID());

  return DoUnregisterVehicle(vehiclePtr);

}

bool BaseConstraintChecker::DoUnregisterVehicle(std::shared_ptr<Vehicle> vehiclePtr){

  return true;

}

void BaseConstraintChecker::CheckConstraint(const MotionCaptureSystemFrame& motionCaptureSystemFrame,
                                            bool emergencyButtonPressed, std::map<std::string, ConstraintCheckerOutput>& checkResult){

  checkResult.clear();

  // do preparation

  map<string, bool> constraintSatisfiedMap;

  DoCheckConstraint(motionCaptureSystemFrame, emergencyButtonPressed, constraintSatisfiedMap);

  VehicleState tmpFrame;

  for(const auto& vehicleResult : constraintSatisfiedMap){

    if(vehicleResult.second == true &&
        motionCaptureSystemFrame.GetFrameElementForVehicle(
            vehicleResult.first, tmpFrame)){

      // store the new pose
      _lastValidStateMap[vehicleResult.first] = tmpFrame;

    }

    checkResult.insert(make_pair(vehicleResult.first,
                                 ConstraintCheckerOutput(vehicleResult.second, _lastValidStateMap[vehicleResult.first])));

  }


}



}