/*
 * trajectory_interface.cpp
 *
 *  Created on: 31.03.2016
 *      Author: burrimi
 */

#include <euroc_stage2/trajectory_interface.h>

namespace euroc_stage2 {

bool TrajectoryInterface::readWaypointsFromFile(const std::string& filename,
                                        std::vector<Eigen::Vector4d>* waypoints,
                                        std::vector<double>* segment_times) {
  std::ifstream wp_file(filename.c_str());

  double previous_total_time = 0;

  int counter = 0;

  if (wp_file.is_open()) {
    double t, x, y, z, yaw;
    while (wp_file >> t >> x >> y >> z >> yaw) {
      Eigen::Vector4d waypoint;
      waypoint << x, y, z, yaw;
      waypoints->push_back(waypoint);
      // Segment times are one less than waypoints.
      if (counter > 0) {
        double segment_time = t - previous_total_time;
        segment_times->push_back(segment_time);
      }
      previous_total_time = t;
      ++counter;
    }
    wp_file.close();
    ROS_INFO("[task2]: Read %d waypoints.", (int)waypoints->size());
  }

  else {
    ROS_ERROR_STREAM("[task2]: Unable to open file: " << filename);
    return false;
  }
  return true;
}

bool TrajectoryInterface::getTrajectoryFromWaypoints(
    const std::vector<Eigen::Vector4d>& waypoints,
    const std::vector<double>& segment_times,
    mav_planning_utils::TrajectoryBase::Ptr* trajectory_position,
    mav_planning_utils::TrajectoryBase::Ptr* trajectory_yaw) {
  mav_planning_utils::Vertex::Vector vertices;
  waypointsToVertices(waypoints, &vertices);

  mav_planning_utils::PolynomialOptimization<kDefaultPolynomialCoefficients>
      linear_optimizer_position(3);
  mav_planning_utils::PolynomialOptimization<kDefaultPolynomialCoefficients>
      linear_optimizer_yaw(1);

  std::vector<mav_planning_utils::Vertex> position_vertices, yaw_vertices;
  vertices4dToPositionAndYaw(vertices, &position_vertices, &yaw_vertices);

  linear_optimizer_position.setupFromVertices(position_vertices, segment_times,
                                              kDerivativeToOptimize);
  linear_optimizer_yaw.setupFromVertices(yaw_vertices, segment_times,
                                         kDerivativeToOptimize);

  linear_optimizer_position.solveLinear();
  linear_optimizer_yaw.solveLinear();

  linear_optimizer_position.getTrajectory(trajectory_position);
  linear_optimizer_yaw.getTrajectory(trajectory_yaw);

  return true;
}

void TrajectoryInterface::vertices4dToPositionAndYaw(
    const std::vector<mav_planning_utils::Vertex>& vertices4d,
    std::vector<mav_planning_utils::Vertex>* position_vertices,
    std::vector<mav_planning_utils::Vertex>* yaw_vertices) {
  // TODO(burrimi): Switch to GLOG.
  if (position_vertices == nullptr || yaw_vertices == nullptr) {
    std::cerr << "[task2]: Got nullptr in vertices4dToPositionAndYaw";
    return;
  }
  position_vertices->clear();
  yaw_vertices->clear();
  for (const mav_planning_utils::Vertex& vertex4d : vertices4d) {
    mav_planning_utils::Vertex pos(3), yaw(1);
    position_vertices->push_back(pos);
    yaw_vertices->push_back(yaw);
    for (mav_planning_utils::Vertex::Constraints::const_iterator it =
             vertex4d.cBegin();
         it != vertex4d.cEnd(); ++it) {
      Eigen::VectorXd tmp_val = it->second.head<3>();
      position_vertices->back().addConstraint(it->first, tmp_val);
      yaw_vertices->back().addConstraint(it->first, it->second[3]);
    }
  }
}

bool TrajectoryInterface::waypointsToVertices(
    const std::vector<Eigen::Vector4d>& waypoints,
    mav_planning_utils::Vertex::Vector* vertices) {
  const size_t n_vertices = waypoints.size();

  vertices->clear();
  vertices->resize(n_vertices, mav_planning_utils::Vertex(4));

  vertices->front().makeStartOrEnd(waypoints.front(), kDerivativeToOptimize);
  vertices->back().makeStartOrEnd(waypoints.back(), kDerivativeToOptimize);

  for (size_t i = 1; i < n_vertices - 1; ++i) {
    mav_planning_utils::Vertex& v = vertices->at(i);
    v.addConstraint(mav_planning_utils::derivative_order::POSITION,
                    waypoints[i]);
  }

  return true;
}

}
