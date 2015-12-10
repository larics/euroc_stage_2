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

namespace VehicleMonitorLibrary {

VehicleMonitor::VehicleMonitor(boost::filesystem::path octoMapFilePath,
                               const Eigen::Vector3d& environmentCorner1,
                               const Eigen::Vector3d& environmentCornerB,
                               unsigned int motionCaptureSystemFrequency)
    : environment_bounding_volume_(environmentCorner1, environmentCornerB),
      motion_capture_system_frequency_(motionCaptureSystemFrequency) {
  if (boost::filesystem::exists(octoMapFilePath) &&
      boost::filesystem::is_regular_file(octoMapFilePath)) {
    _ocTreePtr = std::make_shared<octomap::OcTree>(octoMapFilePath.string());

    octomath::Vector3 vertexMin = environment_bounding_volume_.getVertexMin();
    octomath::Vector3 vertexMax = environment_bounding_volume_.getVertexMax();

    _ocTreePtr->setBBXMin(vertexMin);
    _ocTreePtr->setBBXMax(vertexMax);

    vehicles_map_ = std::make_shared<std::map<std::string, Vehicle::Ptr> >();

  } else {
    throw std::invalid_argument("The OctoMap path is not valid");
  }
}

VehicleMonitor::VehicleMonitor(std::shared_ptr<octomap::OcTree> ocTreePtr,
                               const Eigen::Vector3d& environmentCorner1,
                               const Eigen::Vector3d& environmentCornerB,
                               unsigned int motionCaptureSystemFrequency)
    : _ocTreePtr(ocTreePtr),
      environment_bounding_volume_(environmentCorner1, environmentCornerB),
      motion_capture_system_frequency_(motionCaptureSystemFrequency) {
  octomath::Vector3 vertexMin = environment_bounding_volume_.getVertexMin();
  octomath::Vector3 vertexMax = environment_bounding_volume_.getVertexMax();

  _ocTreePtr->setBBXMin(vertexMin);
  _ocTreePtr->setBBXMax(vertexMax);
}

VehicleMonitor::~VehicleMonitor() {}

std::shared_ptr<octomap::OcTree> VehicleMonitor::getOcTreePtr() {
  return _ocTreePtr;
}

BoundingVolume VehicleMonitor::getEnvironmentBoundingVolume() const {
  return environment_bounding_volume_;
}

std::vector<std::string> VehicleMonitor::getVehicleIDs() const {
  std::vector<std::string> result;

  for (const std::pair<std::string, Vehicle::Ptr>& vehicle_map_element :
       *vehicles_map_) {
    result.push_back(vehicle_map_element.first);
  }

  return result;
}

bool VehicleMonitor::registerChecker(
    BaseConstraintChecker::Ptr constraint_cheker) {
  if (vehicles_map_ == nullptr) {
    std::cout << "[vehicle_monitor_library]: ERROR, vehicles_map_ not set!";
    return false;
  }

  constraint_cheker->setVehiclesMap(vehicles_map_);

  std::pair<std::map<std::string, BaseConstraintChecker::Ptr>::iterator, bool>
      ret;

  ret = constraint_checkers_.insert(
      make_pair(constraint_cheker->getId(), constraint_cheker));

  if (ret.second == false) {
    return false;
  }

  std::cout << "[Vehicle Monitor] Registered constraint checker: "
            << constraint_cheker->getId() << std::endl;

  return true;
}

bool VehicleMonitor::unregisterChecker(
    BaseConstraintChecker::Ptr constraint_cheker) {
  std::map<std::string, BaseConstraintChecker::Ptr>::iterator mapElement;

  mapElement = constraint_checkers_.find(constraint_cheker->getId());

  if (mapElement == constraint_checkers_.end()) {
    return false;
  }

  constraint_checkers_.erase(mapElement);

  std::cout << "[Vehicle Monitor] Unregistered constraint checker: "
            << constraint_cheker->getId() << std::endl;

  return true;
}

void VehicleMonitor::resetAllChecker() {
  for (auto constraintCheckerMapElement : constraint_checkers_) {
    constraintCheckerMapElement.second->reset();
  }
}

bool VehicleMonitor::registerVehicle(Vehicle::Ptr vehicle) {
  std::pair<std::map<std::string, Vehicle::Ptr>::iterator, bool> ret;

  ret = vehicles_map_->insert(make_pair(vehicle->getID(), vehicle));

  if (ret.second == false) {
    return false;
  }

  for (auto constraintCheckerMapElement : constraint_checkers_) {
    constraintCheckerMapElement.second->registerVehicle(vehicle->getID());
  }

  return true;
}

bool VehicleMonitor::unregisterVehicle(Vehicle::Ptr vehicle) {
  std::map<std::string, Vehicle::Ptr>::iterator mapElement;

  mapElement = vehicles_map_->find(vehicle->getID());

  if (mapElement == vehicles_map_->end()) {
    return false;
  }

  vehicles_map_->erase(mapElement);

  for (auto constraintCheckerMapElement : constraint_checkers_) {
    constraintCheckerMapElement.second->unregisterVehicle(vehicle->getID());
  }

  return true;
}

void VehicleMonitor::trigger(
    const MotionCaptureSystemFrame& motionCaptureSystemFrame,
    bool emergencyButtonPressed) {
  last_computed_output_.clear();

  // update velocities of all vehicles.
  for (const std::pair<std::string, Vehicle::Ptr>& vehicleMapElement :
       *vehicles_map_) {
    vehicleMapElement.second->updateState(motionCaptureSystemFrame);
  }

  std::map<std::string, ConstraintCheckerOutput> constraintCheckerResult;

  for (auto constraintMapElement : constraint_checkers_) {
    constraintCheckerResult.clear();

    constraintMapElement.second->checkConstraint(motionCaptureSystemFrame,
                                                 emergencyButtonPressed,
                                                 constraintCheckerResult);

    for (const auto& resultMapElement : constraintCheckerResult) {
      // this means:
      // _lastComputedOutput[vehicleID][constrainCheckerID] =
      // constraincCheckerOutput;
      last_computed_output_[resultMapElement.first][constraintMapElement
                                                        .first] =
          resultMapElement.second;
    }
  }

  notifyObservers(last_computed_output_);
}

bool VehicleMonitor::registerObserver(
    std::shared_ptr<VehicleMonitorObserverBase> observerPtr) {
  std::pair<std::set<std::shared_ptr<VehicleMonitorObserverBase> >::iterator,
            bool> ret;

  ret = observers_.insert(observerPtr);

  if (ret.second == false) {
    return false;
  }

  return true;
}

bool VehicleMonitor::unregisterObserver(
    std::shared_ptr<VehicleMonitorObserverBase> observerPtr) {
  std::set<std::shared_ptr<VehicleMonitorObserverBase> >::iterator setElement;

  setElement = observers_.find(observerPtr);

  if (setElement == observers_.end()) {
    return false;
  }

  observers_.erase(setElement);

  return true;
}

// loop through all registered observers and provide them with the vector of
// outputs
void VehicleMonitor::notifyObservers(
    const std::map<std::string,
                   std::map<std::string, ConstraintCheckerOutput> >&
        vehicleStatus) const {
  for (auto observerPtr : observers_) {
    observerPtr->Update(vehicleStatus);
  }
}
}
