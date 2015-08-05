/**
 *    @file VehicleMonitorLibrary/VehicleState.hpp
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

#ifndef VML__VEHICLE_STATE_H_
#define VML__VEHICLE_STATE_H_

#include <Eigen/Eigen>

#include <octomap/octomap_types.h>

namespace VehicleMonitorLibrary{

struct VehicleState{

  Eigen::Vector3d _linear;
  Eigen::Vector3d _angular;

  VehicleState()
  :_linear(), _angular(){

  }

  VehicleState(const Eigen::Vector3d& linear, const Eigen::Vector3d& angular)
  :_linear(linear), _angular(angular){

  }

};

}
#endif /* VML__VEHICLE_STATE_H_ */
