/**
 *    @file VehicleMonitorLibrary/VehicleMonitor.hpp
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

#ifndef VML__VEHICLE_MONITOR_H_
#define VML__VEHICLE_MONITOR_H_

#include "BoundingVolume.hpp"
#include "ConstraintCheckerOutput.hpp"

#include <Eigen/Eigen>
#include <map>
#include <set>

#include <boost/shared_container_iterator.hpp>
#include <boost/filesystem.hpp>

#include <octomap/octomap_types.h>

namespace octomap{
class OcTree;
}

namespace VehicleMonitorLibrary{

class BaseConstraintChecker;
class Vehicle;
class MotionCaptureSystemFrame;
class VehicleMonitorObserverBase;

class VehicleMonitor{

 public:

  // constructors
  VehicleMonitor(boost::filesystem::path octoMapFilePath,
                 const Eigen::Vector3d& environmentCorner1, const Eigen::Vector3d& environmentCornerB,
                 unsigned int motionCaptureSystemFrequency);
  VehicleMonitor(std::shared_ptr<octomap::OcTree> ocTreePtr,
                 const Eigen::Vector3d& environmentCorner1, const Eigen::Vector3d& environmentCornerB,
                 unsigned int motionCaptureSystemFrequency);
  ~VehicleMonitor();

  // methods
  std::shared_ptr<octomap::OcTree> GetOcTreePtr();

  BoundingVolume GetEnvironmentBoundingVolume() const;

  std::vector<std::string> GetVehicleIDs() const;

  bool RegisterChecker(std::shared_ptr<BaseConstraintChecker> constraintChekerPtr);
  bool UnregisterChecker(std::shared_ptr<BaseConstraintChecker> constraintChekerPtr);

  bool RegisterVehicle(std::shared_ptr<Vehicle> vehiclePtr);
  bool UnregisterVehicle(std::shared_ptr<Vehicle> vehiclePtr);

  bool RegisterObserver(std::shared_ptr<VehicleMonitorObserverBase> observerPtr);
  bool UnregisterObserver(std::shared_ptr<VehicleMonitorObserverBase> observerPtr);

  void Trigger(const MotionCaptureSystemFrame& motionCaptureSystemFrame,
               bool emergencyButtonPressed);


 private :

  void NotifyObservers(const std::map<std::string, std::map<std::string, ConstraintCheckerOutput> >& vehicleStatus) const;

  std::map< std::string, std::shared_ptr<Vehicle> > _vehiclesMap;

  BoundingVolume _environmentBoundingVolume;

  std::map<std::string, std::shared_ptr<BaseConstraintChecker> > _constraintCheckers;

  unsigned int _motionCaptureSystemFrequency;

  std::shared_ptr<octomap::OcTree> _ocTreePtr;

  std::map<std::string, std::map<std::string, ConstraintCheckerOutput> > _lastComputedOutput;

  std::set<std::shared_ptr<VehicleMonitorObserverBase> > _observers;

};

}

#endif /* VML__VEHICLE_MONITOR_H_ */
