/**
 *    @file Vehicle.cpp
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

#include "vehicle_monitor_library/Vehicle.hpp"

#include "vehicle_monitor_library/SimpleVelocityEstimator.hpp"
#include <sstream>

using namespace std;

namespace VehicleMonitorLibrary{


Vehicle::Vehicle(std::string ID, float boundingSphereRadius)
:_ID(ID),
 _boundingSphereRadius(boundingSphereRadius){


}


Vehicle::~Vehicle(){

}

std::string Vehicle::GetID() const{

  return _ID;

}

float Vehicle::GetBoundingSphereRadius() const{

  return _boundingSphereRadius;

}

std::string Vehicle::ToString() const{

  stringstream ss;
  ss << "Vehicle:" << endl << "\tID = " << _ID  << endl << "\tBounding Sphere Radius = " << _boundingSphereRadius << endl;
  return ss.str();

}

}

ostream& operator<< (ostream& outStream, const VehicleMonitorLibrary::Vehicle& vehicle){

  outStream << vehicle.ToString();
  return outStream;

}
