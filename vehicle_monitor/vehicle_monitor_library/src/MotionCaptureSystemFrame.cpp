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

namespace VehicleMonitorLibrary{

MotionCaptureSystemFrame::MotionCaptureSystemFrame(uint64_t frameNumber)
:_frameNumber(frameNumber){

}

MotionCaptureSystemFrame::~MotionCaptureSystemFrame(){

}

bool MotionCaptureSystemFrame::GetFrameElementForVehicle(
    std::string vehicleID,
    VehicleState& output) const{

  map<string, VehicleState>::const_iterator element;
  element = _frameElements.find(vehicleID);

  if(element == _frameElements.end()){
    return false;
  }

  output = element->second;
  return true;

}

bool MotionCaptureSystemFrame::AddFrameElement(std::string vehicleID,
                                               VehicleState frameElement){

  pair<map<string, VehicleState>::iterator,bool> ret;

  ret = _frameElements.insert(make_pair(vehicleID, frameElement));

  if (ret.second==false) {

    return false;

  }

  return true;

}

void MotionCaptureSystemFrame::UpdateFrameElement(std::string vehicleID,
                                                  VehicleState frameElement){

  _frameElements[vehicleID] = frameElement;

}

uint64_t MotionCaptureSystemFrame::GetFrameNumber() const{

  return _frameNumber;

}

void MotionCaptureSystemFrame::SetFrameNumber(uint64_t frameNumber){

  _frameNumber = frameNumber;

}



}
