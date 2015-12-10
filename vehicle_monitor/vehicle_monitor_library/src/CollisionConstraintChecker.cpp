/**
 *    @file CollisionConstraintChecker.cpp
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

#include "vehicle_monitor_library/CollisionConstraintChecker.hpp"

#include "vehicle_monitor_library/Vehicle.hpp"
#include "vehicle_monitor_library/VehicleMonitor.hpp"
#include "vehicle_monitor_library/BoundingVolume.hpp"
#include "vehicle_monitor_library/BaseVelocityEstimator.hpp"
#include "vehicle_monitor_library/MotionCaptureSystemFrame.hpp"

using namespace std;

namespace VehicleMonitorLibrary {

CollisionConstraintChecker::CollisionConstraintChecker(
    std::shared_ptr<octomap::OcTree> oc_tree,
    const BoundingVolume& environment_bounding_volume,
    double max_dist_to_check_collision,
    double collision_threshold_in_bounding_sphere_radius,
    unsigned int projection_window,
    unsigned int motion_capture_system_frequency, double minimum_height)
    : BaseConstraintChecker("COLLISION_CONSTRAINT_CHECKER"),
      oc_tree_ptr_(oc_tree),
      dist_map_(max_dist_to_check_collision, oc_tree_ptr_.get(),
                environment_bounding_volume.getVertexMin(),
                environment_bounding_volume.getVertexMax(), true),
      collision_threshold_in_bounding_sphere_radius_(
          collision_threshold_in_bounding_sphere_radius),
      projection_window_size_(projection_window),
      minimum_height_(minimum_height),
      motion_capture_system_frequency_(motion_capture_system_frequency) {
  dist_map_.update();
}

CollisionConstraintChecker::~CollisionConstraintChecker() {}

void CollisionConstraintChecker::doCheckConstraint(
    const MotionCaptureSystemFrame& motion_capture_system_frame,
    bool emergency_button_pressed, std::map<std::string, bool>& check_result) {
  bool constraint_ok;

  VehicleState estimated_state;

  double delta_time =
      projection_window_size_ / (double)motion_capture_system_frequency_;

  for (const std::pair<std::string, Vehicle::Ptr>& vehicle_map_element :
       *vehicles_map_) {
    const std::string& vehicle_id = vehicle_map_element.first;
    Vehicle::Ptr vehicle = vehicle_map_element.second;
    if (vehicle->getHasNewState() == false) {
      continue;
    }

    if (vehicle->getState(&estimated_state) == false) {
      continue;
    }

    if (take_off_detected_.find(vehicle_id) == take_off_detected_.end()) {
      if (estimated_state.position.z() > minimum_height_) {
        take_off_detected_.insert(vehicle_map_element.first);
        std::cout << "[VEHICLE_MONITOR] Collision Constraint activated for "
                  << vehicle_id << std::endl;
      } else {
        continue;
      }
    }

    // Project current position according to velocity estimation
    Eigen::Vector3d future_pose(estimated_state.position.x() +
                                    estimated_state.velocity.x() * delta_time,
                                estimated_state.position.y() +
                                    estimated_state.velocity.y() * delta_time,
                                estimated_state.position.z() +
                                    estimated_state.velocity.z() * delta_time);

    // Given the future position check for a collision
    octomath::Vector3 future_pose_octomath(future_pose.x(), future_pose.y(),
                                           future_pose.z());

    double distance = dist_map_.getDistance(future_pose_octomath);

    if (distance == DynamicEDTOctomap::distanceValue_Error) {
      std::cout << "VEHICLE NOT IN MAP" << std::endl;
      constraint_ok = false;
    } else {
      constraint_ok =
          distance > (collision_threshold_in_bounding_sphere_radius_ *
                      vehicle->getBoundingSphereRadius());
    }
    check_result.insert(make_pair(vehicle_id, constraint_ok));
  }
}

void CollisionConstraintChecker::doReset() {
  take_off_detected_.clear();
}

}
