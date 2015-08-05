/**
 *    @file OutOfSpaceConstraintChecker.cpp
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

#include "vehicle_monitor_library/OutOfSpaceConstraintChecker.hpp"

#include "vehicle_monitor_library/Vehicle.hpp"
#include "vehicle_monitor_library/MotionCaptureSystemFrame.hpp"

using namespace std;

namespace VehicleMonitorLibrary{


OutOfSpaceConstraintChecker::OutOfSpaceConstraintChecker(BoundingVolume environmentBoundingVolume)
:BaseConstraintChecker("OUT_OF_SPACE_CONSTRAINT_CHECKER"),
 _environmentBoundingVolume(environmentBoundingVolume){

}

OutOfSpaceConstraintChecker::~OutOfSpaceConstraintChecker(){

}

void OutOfSpaceConstraintChecker::DoCheckConstraint(const MotionCaptureSystemFrame& motionCaptureSystemFrame,
                                                    bool emergencyButtonPressed, std::map<std::string, bool>& checkResult) const{


  bool vehicleResult;

  VehicleState frameElement;

  for(const auto vehicleMapElement : _vehiclesMap){

    if( motionCaptureSystemFrame.GetFrameElementForVehicle(vehicleMapElement.first, frameElement) == false){
      continue;
    }

    checkResult.insert(make_pair(vehicleMapElement.first,
                                 _environmentBoundingVolume.IsSphereInsideBB(frameElement._linear,
                                                                             vehicleMapElement.second->GetBoundingSphereRadius())));

  }

}


}
