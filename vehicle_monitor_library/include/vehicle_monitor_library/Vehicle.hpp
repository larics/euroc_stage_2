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

namespace VehicleMonitorLibrary{
class Vehicle;
}

std::ostream& operator<< (std::ostream& outStream, const VehicleMonitorLibrary::Vehicle& vehicle);

namespace VehicleMonitorLibrary{

class IVelocityEstimator;
class MotionCaptureSystemFrame;

class Vehicle{

 public:

  Vehicle(std::string ID, float boundingSphereRadius);

  ~Vehicle();

  std::string GetID() const;

  float GetBoundingSphereRadius() const;

  void UpdatePose(const MotionCaptureSystemFrame& motionCaptureSystemFrame);

  std::string ToString() const;

 private:

  std::string _ID;                     // id used to identify and access vehicle in the VehicleContainer

  float _boundingSphereRadius;

};

}
#endif /* VML__VEHICLE_H_ */
