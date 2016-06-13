/**
 *    @file VehicleMonitorLibrary/CollisionConstraintChecker.hpp
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

#ifndef VML__COLLISION_CONSTRAINT_CHECKER_H_
#define VML__COLLISION_CONSTRAINT_CHECKER_H_

#include <set>
#include <unordered_set>

#include "BaseConstraintChecker.hpp"

#include "BoundingVolume.hpp"

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

namespace octomap {
class OcTree;
}

namespace VehicleMonitorLibrary {

// small helper class to allow messages to change octree without changing
// pointer address of object
class OctreeHolder {
 public:
  OctreeHolder(void);

  octomap::OcTree* getOctreePtr(void);

  void updateOctree(const octomap_msgs::OctomapConstPtr& msg);

 private:
  octomap::OcTree* octree_ptr_;
};

class CollisionConstraintChecker : public BaseConstraintChecker {
 public:
  CollisionConstraintChecker(std::shared_ptr<OctreeHolder> octree_holder_ptr,
                             double collision_threshold_distance,
                             double vehicle_height, double vehicle_radius,
                             unsigned int projection_window,
                             unsigned int motion_capture_system_frequency,
                             double minimum_height);

  virtual ~CollisionConstraintChecker();

 protected:
  virtual void doCheckConstraint(
      const MotionCaptureSystemFrame& motion_capture_system_frame,
      bool emergency_button_pressed, std::map<std::string, bool>& check_result);

  void doReset();

 private:
  std::shared_ptr<OctreeHolder> octree_holder_ptr_;

  double collision_threshold_distance_;

  double vehicle_radius_;

  double vehicle_height_;

  // number of time steps used to compute the future position of a vehicle
  unsigned int projection_window_size_;

  unsigned int motion_capture_system_frequency_;

  double minimum_height_;

  std::unordered_set<std::string> take_off_detected_;
};
}
#endif /* VML__COLLISION_CONSTRAINT_CHECKER_H_ */
