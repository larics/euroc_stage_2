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

#include <boost/filesystem.hpp>
#include <Eigen/Eigen>
#include <map>
#include <set>

#include <octomap/octomap_types.h>

#include "vehicle_monitor_library/BaseConstraintChecker.hpp"
#include "vehicle_monitor_library/BoundingVolume.hpp"
#include "vehicle_monitor_library/ConstraintCheckerOutput.hpp"
#include "vehicle_monitor_library/Vehicle.hpp"

namespace octomap {
class OcTree;
}

namespace VehicleMonitorLibrary {

class MotionCaptureSystemFrame;
class VehicleMonitorObserverBase;

class VehicleMonitor {
 public:
  // constructors
  VehicleMonitor(boost::filesystem::path octoMapFilePath,
                 const Eigen::Vector3d& environmentCorner1,
                 const Eigen::Vector3d& environmentCornerB,
                 unsigned int motionCaptureSystemFrequency);
  VehicleMonitor(std::shared_ptr<octomap::OcTree> ocTreePtr,
                 const Eigen::Vector3d& environmentCorner1,
                 const Eigen::Vector3d& environmentCornerB,
                 unsigned int motionCaptureSystemFrequency);
  ~VehicleMonitor();

  // methods
  std::shared_ptr<octomap::OcTree> GetOcTreePtr();

  BoundingVolume GetEnvironmentBoundingVolume() const;

  std::vector<std::string> GetVehicleIDs() const;

  bool RegisterChecker(BaseConstraintChecker::Ptr constraint_cheker);
  bool UnregisterChecker(BaseConstraintChecker::Ptr constraint_cheker);

  bool RegisterVehicle(Vehicle::Ptr vehicle);
  bool UnregisterVehicle(Vehicle::Ptr vehicle);

  bool RegisterObserver(std::shared_ptr<VehicleMonitorObserverBase> observer);
  bool UnregisterObserver(std::shared_ptr<VehicleMonitorObserverBase> observer);

  void Trigger(const MotionCaptureSystemFrame& motion_capture_system_frame,
               bool emergency_button_pressed);

 private:
  void NotifyObservers(
      const std::map<std::string,
                     std::map<std::string, ConstraintCheckerOutput> >&
          vehicle_status) const;

  std::shared_ptr<std::map<std::string, Vehicle::Ptr> > vehicles_map_;

  BoundingVolume environment_bounding_volume_;

  std::map<std::string, BaseConstraintChecker::Ptr> constraint_checkers_;

  unsigned int motion_capture_system_frequency_;

  std::shared_ptr<octomap::OcTree> _ocTreePtr;

  std::map<std::string, std::map<std::string, ConstraintCheckerOutput> >
      last_computed_output_;

  std::set<std::shared_ptr<VehicleMonitorObserverBase> > observers_;
};
}

#endif /* VML__VEHICLE_MONITOR_H_ */
