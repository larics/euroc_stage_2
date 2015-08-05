/**
 *    @file VehicleMonitorLibrary/VehicleMonitorObserver.hpp
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

#ifndef VML__VEHICLE_MONITOR_OBSERVER_H_
#define VML__VEHICLE_MONITOR_OBSERVER_H_

#include <map>
#include <string>

#include "ConstraintCheckerOutput.hpp"

namespace VehicleMonitorLibrary{

struct VMVehicleStatus;

class VehicleMonitorObserverBase{

 public:

  VehicleMonitorObserverBase(){};

  virtual ~VehicleMonitorObserverBase(){};

  // firs string vehicle ID, second  string constraint checker ID;

  virtual void Update(const std::map<std::string, std::map<std::string, ConstraintCheckerOutput> >& vehicleStatus) = 0;


};

}

#endif /* VML__VEHICLE_MONITOR_OBSERVER_H_ */
