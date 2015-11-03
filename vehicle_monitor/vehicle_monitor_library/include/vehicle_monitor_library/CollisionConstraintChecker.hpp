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

#include <unordered_set>
#include <set>

#include "BaseConstraintChecker.hpp"

#include "BoundingVolume.hpp"

#include "octomap/octomap_types.h"
#include "dynamicEDT3D/dynamicEDTOctomap.h"

namespace octomap {
class OcTree;
}

namespace VehicleMonitorLibrary {

class CollisionConstraintChecker : public BaseConstraintChecker {
 public:
  CollisionConstraintChecker(
      std::shared_ptr<octomap::OcTree> oc_tree,
      const BoundingVolume& environment_bounding_volume,
      double max_dist_to_check_collision,
      double collision_threeshold_in_bounding_sphere_radius,
      unsigned int projection_window,
      unsigned int motion_capture_system_frequency, double minimum_height);

  virtual ~CollisionConstraintChecker();

 protected:
  virtual void doCheckConstraint(
      const MotionCaptureSystemFrame& motion_capture_system_frame,
      bool emergency_button_pressed, std::map<std::string, bool>& check_result);

 private:
  std::shared_ptr<octomap::OcTree> oc_tree_ptr_;

  DynamicEDTOctomap dist_map_;

  double collision_threshold_in_bounding_sphere_radius_;

  // number of time steps used to compute the future position of a vehicle
  unsigned int projection_window_size_;

  unsigned int motion_capture_system_frequency_;

  double minimum_height_;

  std::unordered_set<std::string> take_off_detected_;
};
}
#endif /* VML__COLLISION_CONSTRAINT_CHECKER_H_ */
