/**
 *    @file MotionCaptureSystemFrame.cpp
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

#include "vehicle_monitor_library/MotionCaptureSystemFrame.hpp"

using namespace std;

namespace VehicleMonitorLibrary {

MotionCaptureSystemFrame::MotionCaptureSystemFrame(uint64_t frame_number)
    : frame_number_(frame_number) {}

MotionCaptureSystemFrame::~MotionCaptureSystemFrame() {}

bool MotionCaptureSystemFrame::getFrameElementForVehicle(
    std::string vehicle_id, VehicleState& output) const {
  map<string, VehicleState>::const_iterator element;
  element = frame_elements_.find(vehicle_id);

  if (element == frame_elements_.end()) {
    return false;
  }

  output = element->second;
  return true;
}

bool MotionCaptureSystemFrame::addFrameElement(std::string vehicle_id,
                                               VehicleState frame_element) {
  pair<map<string, VehicleState>::iterator, bool> ret;

  ret = frame_elements_.insert(make_pair(vehicle_id, frame_element));

  if (ret.second == false) {
    return false;
  }

  return true;
}

void MotionCaptureSystemFrame::updateFrameElement(std::string vehicle_id,
                                                  VehicleState frame_element) {
  frame_elements_[vehicle_id] = frame_element;
}

uint64_t MotionCaptureSystemFrame::getFrameNumber() const {
  return frame_number_;
}

void MotionCaptureSystemFrame::setFrameNumber(uint64_t frame_number) {
  frame_number_ = frame_number;
}
}
