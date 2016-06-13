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
#include "vehicle_monitor_library/CollisionConstraintChecker.hpp"
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
  VehicleMonitor(std::shared_ptr<OctreeHolder> ocTreePtr,
                 const Eigen::Vector3d& environmentCorner1,
                 const Eigen::Vector3d& environmentCornerB,
                 unsigned int motionCaptureSystemFrequency);
  ~VehicleMonitor();

  // methods
  std::shared_ptr<OctreeHolder> getOcTreePtr();

  BoundingVolume getEnvironmentBoundingVolume() const;

  std::vector<std::string> getVehicleIDs() const;

  bool registerChecker(BaseConstraintChecker::Ptr constraint_cheker);
  bool unregisterChecker(BaseConstraintChecker::Ptr constraint_cheker);
  void resetAllChecker();

  bool registerVehicle(Vehicle::Ptr vehicle);
  bool unregisterVehicle(Vehicle::Ptr vehicle);

  bool registerObserver(std::shared_ptr<VehicleMonitorObserverBase> observer);
  bool unregisterObserver(std::shared_ptr<VehicleMonitorObserverBase> observer);

  void trigger(const MotionCaptureSystemFrame& motion_capture_system_frame,
               bool emergency_button_pressed);

 private:
  void notifyObservers(
      const std::map<std::string,
                     std::map<std::string, ConstraintCheckerOutput> >&
          vehicle_status) const;

  std::shared_ptr<std::map<std::string, Vehicle::Ptr> > vehicles_map_;

  BoundingVolume environment_bounding_volume_;

  std::map<std::string, BaseConstraintChecker::Ptr> constraint_checkers_;

  unsigned int motion_capture_system_frequency_;

  std::shared_ptr<OctreeHolder> _ocTreePtr;

  std::map<std::string, std::map<std::string, ConstraintCheckerOutput> >
      last_computed_output_;

  std::set<std::shared_ptr<VehicleMonitorObserverBase> > observers_;
};
}

#endif /* VML__VEHICLE_MONITOR_H_ */
