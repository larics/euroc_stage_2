/**
 *    @file VehicleMonitorLibrary/BaseConstraintChecker.hpp
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

#ifndef VML__BASE_CONSTRAINT_CHECKER_H_
#define VML__BASE_CONSTRAINT_CHECKER_H_

#include <map>
#include <string>

#include <boost/shared_ptr.hpp>

#include <octomap/octomap_types.h>

#include "ConstraintCheckerOutput.hpp"
#include "VehicleState.hpp"

namespace VehicleMonitorLibrary{

class Vehicle;
class MotionCaptureSystemFrame;

class BaseConstraintChecker {

 public:

  BaseConstraintChecker(std::string ID);

  virtual ~BaseConstraintChecker();

  std::string GetID() const;

  bool RegisterVehicle(std::shared_ptr<Vehicle> vehiclePtr);

  bool UnregisterVehicle(std::shared_ptr<Vehicle> vehiclePtr);

  void CheckConstraint(const MotionCaptureSystemFrame& motionCaptureSystemFrame,
                       bool emergencyButtonPressed, std::map<std::string, ConstraintCheckerOutput>& checkResult);

 protected:

  virtual bool DoRegisterVehicle(std::shared_ptr<Vehicle> vehiclePtr);

  virtual bool DoUnregisterVehicle(std::shared_ptr<Vehicle> vehiclePtr);

  virtual void DoCheckConstraint(const MotionCaptureSystemFrame& motionCaptureSystemFrame,
                                 bool emergencyButtonPressed, std::map<std::string, bool>& checkResult)  const = 0;

  std::string _ID;

  std::map< std::string, std::shared_ptr<Vehicle> > _vehiclesMap;
  std::map< std::string, VehicleState > _lastValidStateMap;

};

}
#endif /* VML__BASE_CONSTRAINT_CHECKER_H_ */
