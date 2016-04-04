/**
 *    @file VehicleMonitorLibrary/Vehicle.hpp
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

#ifndef VML__VEHICLE_H_
#define VML__VEHICLE_H_

#include <vector>
#include <string>
#include <ostream>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include "vehicle_monitor_library/BaseVelocityEstimator.hpp"
#include "vehicle_monitor_library/MotionCaptureSystemFrame.hpp"

namespace VehicleMonitorLibrary {
class Vehicle;
}

std::ostream& operator<<(std::ostream& outStream,
                         const VehicleMonitorLibrary::Vehicle& vehicle);

namespace VehicleMonitorLibrary {

class MotionCaptureSystemFrame;

class Vehicle {
 public:
  typedef std::shared_ptr<Vehicle> Ptr;

  Vehicle(const std::string& id, double bounding_sphere_radius,
          BaseVelocityEstimator::Ptr velocity_estimator);

  ~Vehicle();

  std::string getID() const;

  double getBoundingSphereRadius() const;

  std::string toString() const;

  void updateState(const MotionCaptureSystemFrame& motion_capture_system_frame);
  bool getState(VehicleState* estimated_velocity_state);

  bool getHasNewState() { return has_new_state_; }
  void resetHasNewState() { has_new_state_ = false; }

 private:
  std::string
      id_;  // id used to identify and access vehicle in the VehicleContainer
  double bounding_sphere_radius_;

  BaseVelocityEstimator::Ptr velocity_estimator_;
  VehicleState estimated_state_;
  bool valid_velocity_;
  bool has_new_state_;
};
}
#endif /* VML__VEHICLE_H_ */
