/**
 *    @file BaseVelocityEstimator.cpp
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

#include "vehicle_monitor_library/BaseVelocityEstimator.hpp"

using namespace std;

namespace VehicleMonitorLibrary{

BaseVelocityEstimator::BaseVelocityEstimator(){

}

BaseVelocityEstimator::~BaseVelocityEstimator(){

}

bool BaseVelocityEstimator::RegisterVehicle(std::string vehicleID){

  pair<set<string>::iterator, bool> ret;

  ret = _vehicleIDs.insert(vehicleID);

  if (ret.second==false) {

    return false;

  }

  if(DoRegisterVehicle(vehicleID) == false){

    _vehicleIDs.erase(vehicleID);

  }

  return true;

}

bool BaseVelocityEstimator::DoRegisterVehicle(std::string vehicleID){

  return true;

}

bool BaseVelocityEstimator::UnregisterVehicle(std::string vehicleID){

  set<string>::iterator setElement;

  setElement = _vehicleIDs.find(vehicleID);

  if(setElement == _vehicleIDs.end()){
    return false;
  }

  _vehicleIDs.erase(setElement);

  return DoUnregisterVehicle(vehicleID);

}
bool BaseVelocityEstimator::DoUnregisterVehicle(std::string vehicleID){

  return true;

}


}