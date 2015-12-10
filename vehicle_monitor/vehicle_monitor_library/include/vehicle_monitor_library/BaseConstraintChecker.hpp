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
#include "vehicle_monitor_library/Vehicle.hpp"

namespace VehicleMonitorLibrary {

class MotionCaptureSystemFrame;

class BaseConstraintChecker {
 public:
  typedef std::shared_ptr<BaseConstraintChecker> Ptr;

  BaseConstraintChecker(std::string constraint_id);

  virtual ~BaseConstraintChecker();

  std::string getId() const;

  bool registerVehicle(const std::string& vehicle_id);

  bool unregisterVehicle(const std::string& vehicle_id);

  void checkConstraint(
      const MotionCaptureSystemFrame& motion_capture_system_frame,
      bool emergency_button_pressed,
      std::map<std::string, ConstraintCheckerOutput>& check_result);

  void setVehiclesMap(
      std::shared_ptr<std::map<std::string, Vehicle::Ptr> > vehicles_map);

  void reset();

 protected:
  // Optional function for the constraints, to get notified about new vehicles.
  virtual bool doRegisterVehicle(const std::string& vehicle_id);
  // Optional function for the constraints, to get notified about removed
  // vehicles.
  virtual bool doUnregisterVehicle(const std::string& vehicle_id);

  virtual void doCheckConstraint(
      const MotionCaptureSystemFrame& motionCaptureSystemFrame,
      bool emergencyButtonPressed,
      std::map<std::string, bool>& checkResult) = 0;

  virtual void doReset();

  std::string constraint_id_;

  std::shared_ptr<std::map<std::string, Vehicle::Ptr> > vehicles_map_;

  std::map<std::string, VehicleState> last_valid_state_map_;
};
}
#endif /* VML__BASE_CONSTRAINT_CHECKER_H_ */
