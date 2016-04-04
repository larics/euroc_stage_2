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

namespace VehicleMonitorLibrary {

struct VehicleState {
 public:
  Eigen::Vector3d position;
  Eigen::Vector3d orientation;

  Eigen::Vector3d velocity;
  Eigen::Vector3d angular_rate;

  bool velocity_valid;

  VehicleState()
      : position(Eigen::Vector3d::Zero()),
        orientation(Eigen::Vector3d::Zero()),
        velocity(Eigen::Vector3d::Zero()),
        angular_rate(Eigen::Vector3d::Zero()),
        velocity_valid(false) {}

  VehicleState(const Eigen::Vector3d& position,
               const Eigen::Vector3d& orientation)
      : position(position),
        orientation(orientation),
        velocity(Eigen::Vector3d::Zero()),
        angular_rate(Eigen::Vector3d::Zero()),
        velocity_valid(false) {}

  VehicleState(const Eigen::Vector3d& position,
               const Eigen::Vector3d& orientation,
               const Eigen::Vector3d& velocity,
               const Eigen::Vector3d& angular_rate)
      : position(position),
        orientation(orientation),
        velocity(velocity),
        angular_rate(angular_rate),
        velocity_valid(true) {}

  void copyVelocityAndRate(const VehicleState& state) {
    velocity = state.velocity;
    angular_rate = state.angular_rate;
    velocity_valid = state.velocity_valid;
  }

  void reset() {
    position = Eigen::Vector3d::Zero();
    orientation = Eigen::Vector3d::Zero();
    velocity = Eigen::Vector3d::Zero();
    angular_rate = Eigen::Vector3d::Zero();
  }
};
}
#endif /* VML__VEHICLE_STATE_H_ */
