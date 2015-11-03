/**
 *    @file VehicleMonitorLibrary/MotionCaptureSystemFrame.hpp
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

#ifndef VML__MOTION_CAPTURE_SYSTEM_FRAME_H_
#define VML__MOTION_CAPTURE_SYSTEM_FRAME_H_

#include <octomap/octomap_types.h>
#include <map>
#include <string>
#include <stdint.h>

#include "vehicle_monitor_library/VehicleState.hpp"

namespace VehicleMonitorLibrary {

class MotionCaptureSystemFrame {
 public:
  MotionCaptureSystemFrame(uint64_t frame_number);
  ~MotionCaptureSystemFrame();

  bool getFrameElementForVehicle(std::string vehicle_id,
                                 VehicleState& output) const;

  bool addFrameElement(std::string vehicle_id, VehicleState frame_element);

  // If not present insert, otherwise update
  void updateFrameElement(std::string vehicle_id, VehicleState frame_element);

  uint64_t getFrameNumber() const;

  void setFrameNumber(uint64_t frame_number);

 private:
  uint64_t frame_number_;
  std::map<std::string, VehicleState> frame_elements_;
};
}

#endif /* VML__MOTION_CAPTURE_SYSTEM_FRAME_H_*/
