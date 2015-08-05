/**
 *    @file VehicleMonitor.cpp
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



#include <octomap/OcTree.h>

#include "vehicle_monitor_library/BaseConstraintChecker.hpp"
#include "vehicle_monitor_library/Vehicle.hpp"
#include "vehicle_monitor_library/VehicleMonitor.hpp"
#include "vehicle_monitor_library/VehicleMonitorObserver.hpp"

namespace VehicleMonitorLibrary{


VehicleMonitor::VehicleMonitor(boost::filesystem::path octoMapFilePath,
                               const Eigen::Vector3d& environmentCorner1, const Eigen::Vector3d& environmentCornerB,
                               unsigned int motionCaptureSystemFrequency)
: _environmentBoundingVolume(environmentCorner1, environmentCornerB),
  _motionCaptureSystemFrequency(motionCaptureSystemFrequency){

  if(boost::filesystem::exists(octoMapFilePath) &&
      boost::filesystem::is_regular_file(octoMapFilePath)){

    _ocTreePtr = std::make_shared<octomap::OcTree>(octoMapFilePath.string());

    octomath::Vector3 vertexMin = _environmentBoundingVolume.GetVertexMin();
    octomath::Vector3 vertexMax = _environmentBoundingVolume.GetVertexMax();

    _ocTreePtr->setBBXMin(vertexMin);
    _ocTreePtr->setBBXMax(vertexMax);

  }else{

    throw std::invalid_argument("The OctoMap path is not valid");

  }

}

VehicleMonitor::VehicleMonitor(std::shared_ptr<octomap::OcTree> ocTreePtr,
                               const Eigen::Vector3d& environmentCorner1, const Eigen::Vector3d& environmentCornerB,
                               unsigned int motionCaptureSystemFrequency)
:_ocTreePtr(ocTreePtr),
 _environmentBoundingVolume(environmentCorner1, environmentCornerB),
 _motionCaptureSystemFrequency(motionCaptureSystemFrequency){

  octomath::Vector3 vertexMin = _environmentBoundingVolume.GetVertexMin();
  octomath::Vector3 vertexMax = _environmentBoundingVolume.GetVertexMax();

  _ocTreePtr->setBBXMin(vertexMin);
  _ocTreePtr->setBBXMax(vertexMax);

}


VehicleMonitor::~VehicleMonitor(){

}

std::shared_ptr<octomap::OcTree>  VehicleMonitor::GetOcTreePtr() {

  return _ocTreePtr;

}

BoundingVolume VehicleMonitor::GetEnvironmentBoundingVolume() const{

  return _environmentBoundingVolume;

}

std::vector<std::string> VehicleMonitor::GetVehicleIDs() const{

  std::vector<std::string> result;

  for(auto vehicleMapElement : _vehiclesMap){

    result.push_back(vehicleMapElement.first);

  }

  return result;


}

bool VehicleMonitor::RegisterChecker(std::shared_ptr<BaseConstraintChecker> constraintChekerPtr){

  std::pair<std::map<std::string, std::shared_ptr<BaseConstraintChecker> >::iterator,bool> ret;

  ret = _constraintCheckers.insert(make_pair(constraintChekerPtr->GetID(), constraintChekerPtr));

  if (ret.second==false) {

    return false;

  }

  std::cout << "[Vehicle Monitor] Registered constraint checker: " << constraintChekerPtr->GetID() << std::endl;

  return true;

}

bool VehicleMonitor::UnregisterChecker(std::shared_ptr<BaseConstraintChecker> constraintChekerPtr){

  std::map<std::string, std::shared_ptr<BaseConstraintChecker> >::iterator mapElement;

  mapElement = _constraintCheckers.find(constraintChekerPtr->GetID());

  if(mapElement == _constraintCheckers.end()){
    return false;
  }

  _constraintCheckers.erase(mapElement);

  std::cout << "[Vehicle Monitor] Unregistered constraint checker: " << constraintChekerPtr->GetID() << std::endl;

  return true;

}

bool VehicleMonitor::RegisterVehicle(std::shared_ptr<Vehicle> vehiclePtr){

  std::pair<std::map<std::string, std::shared_ptr<Vehicle> >::iterator,bool> ret;

  ret = _vehiclesMap.insert(make_pair(vehiclePtr->GetID(), vehiclePtr));

  if (ret.second==false) {

    return false;

  }

  for(auto constraintCheckerMapElement : _constraintCheckers){

    constraintCheckerMapElement.second->RegisterVehicle(vehiclePtr);

  }

  return true;

}

bool VehicleMonitor::UnregisterVehicle(std::shared_ptr<Vehicle> vehiclePtr){

  std::map<std::string, std::shared_ptr<Vehicle> >::iterator mapElement;

  mapElement = _vehiclesMap.find(vehiclePtr->GetID());

  if(mapElement == _vehiclesMap.end()){
    return false;
  }

  _vehiclesMap.erase(mapElement);

  for(auto constraintCheckerMapElement : _constraintCheckers){

    constraintCheckerMapElement.second->UnregisterVehicle(vehiclePtr);

  }

  return true;

}

void VehicleMonitor::Trigger(const MotionCaptureSystemFrame& motionCaptureSystemFrame,
                             bool emergencyButtonPressed){

  _lastComputedOutput.clear();

  std::map<std::string, ConstraintCheckerOutput> constraintCheckerResult;

  for(auto constraintMapElement : _constraintCheckers){

    constraintCheckerResult.clear();

    constraintMapElement.second->CheckConstraint(motionCaptureSystemFrame,
                                                 emergencyButtonPressed, constraintCheckerResult);

    for(const auto& resultMapElement : constraintCheckerResult){

      // this means:
      // _lastComputedOutput[vehicleID][constrainCheckerID] = constraincCheckerOutput;
      _lastComputedOutput[resultMapElement.first][constraintMapElement.first] = resultMapElement.second;

    }

  }

  NotifyObservers(_lastComputedOutput);

}

bool VehicleMonitor::RegisterObserver(std::shared_ptr<VehicleMonitorObserverBase> observerPtr) {

  std::pair<std::set<std::shared_ptr<VehicleMonitorObserverBase> >::iterator,bool> ret;

  ret = _observers.insert(observerPtr);

  if (ret.second==false) {

    return false;

  }

  return true;

}

bool VehicleMonitor::UnregisterObserver(std::shared_ptr<VehicleMonitorObserverBase> observerPtr) {

  std::set<std::shared_ptr<VehicleMonitorObserverBase> >::iterator setElement;

  setElement = _observers.find(observerPtr);

  if(setElement == _observers.end()){
    return false;
  }

  _observers.erase(setElement);

  return true;

}

// loop through all registered observers and provide them with the vector of outputs
void VehicleMonitor::NotifyObservers(const std::map<std::string,
                                     std::map<std::string, ConstraintCheckerOutput> >& vehicleStatus) const{

  for(auto observerPtr : _observers){

    observerPtr->Update(vehicleStatus);

  }

}

}
