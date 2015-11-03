/**
 *    @file VehicleMonitorLibrary/MotionConstraintChecker.hpp
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

#ifndef VML__MOTION_CONSTRAINT_CHECKER_H_
#define VML__MOTION_CONSTRAINT_CHECKER_H_

#include "BaseConstraintChecker.hpp"

namespace VehicleMonitorLibrary {

class MotionConstraintChecker : public BaseConstraintChecker {
 public:
  MotionConstraintChecker();

  virtual ~MotionConstraintChecker();

 protected:
  virtual void doCheckConstraint(
      const MotionCaptureSystemFrame& motionCaptureSystemFrame,
      bool emergency_button_pressed, std::map<std::string, bool>& checkResult);

 private:
};
}

#endif /* VML__MOTION_CONSTRAINT_CHECKER_H_ */
