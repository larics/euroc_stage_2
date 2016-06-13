/*
 * trajectory_interface.h
 *
 *  Created on: 31.03.2016
 *      Author: burrimi
 */

#ifndef INCLUDE_EUROC_STAGE2_TRAJECTORY_INTERFACE_H_
#define INCLUDE_EUROC_STAGE2_TRAJECTORY_INTERFACE_H_

#include <Eigen/Eigen>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>

#include <mav_planning_utils/polynomial_optimization_nonlinear.h>
#include <mav_planning_utils/trajectory_base.h>
#include <mav_planning_utils/trajectory_types.h>
#include <mav_planning_utils/trajectory_base.h>
#include <mav_planning_utils/ros_trajectory_interface.h>

namespace euroc_stage2 {


class TrajectoryInterface {
 public:
  static constexpr int kDefaultPolynomialCoefficients = 10;
  static constexpr int kDerivativeToOptimize =
      mav_planning_utils::derivative_order::SNAP;

  TrajectoryInterface() {}
  ~TrajectoryInterface() {}

 public:
  static bool readWaypointsFromFile(const std::string& filename,
                             std::vector<Eigen::Vector4d>* waypoints,
                             std::vector<double>* segment_times);

  static bool getTrajectoryFromWaypoints(
      const std::vector<Eigen::Vector4d>& waypoints,
      const std::vector<double>& segment_times,
      mav_planning_utils::TrajectoryBase::Ptr* trajectory_position,
      mav_planning_utils::TrajectoryBase::Ptr* trajectory_yaw);

  static void vertices4dToPositionAndYaw(
      const std::vector<mav_planning_utils::Vertex>& vertices4d,
      std::vector<mav_planning_utils::Vertex>* position_vertices,
      std::vector<mav_planning_utils::Vertex>* yaw_vertices);

  static bool waypointsToVertices(const std::vector<Eigen::Vector4d>& waypoints,
                           mav_planning_utils::Vertex::Vector* vertices);

};

}

#endif /* INCLUDE_EUROC_STAGE2_TRAJECTORY_INTERFACE_H_ */
