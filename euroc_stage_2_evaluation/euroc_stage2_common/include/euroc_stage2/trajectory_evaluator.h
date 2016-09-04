#ifndef EUROC_STAGE2_COMMON_TRAJECTORY_EVALUATOR_H
#define EUROC_STAGE2_COMMON_TRAJECTORY_EVALUATOR_H

#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/MarkerArray.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <Eigen/Eigen>

namespace euroc_stage2 {

class TrajectoryEvaluator {
 public:
  TrajectoryEvaluator(double trajectory_start_distance, double trajectory_end_distance);

  // returns true if new point results in a new minimum error
  bool addPoint(const ros::Duration& point_time, const Eigen::Vector3d& point_position);

  void trajectoryCallback(trajectory_msgs::MultiDOFJointTrajectoryConstPtr msg);

  visualization_msgs::MarkerArray getMarkers() const;

  double getMinError() const;

  ros::Duration getTimeOffsetFromStart() const;

 private:
  static constexpr double kVicondt = 0.01;

  ros::Subscriber trajectory_sub_;
  std::vector<std::pair<ros::Duration, Eigen::Vector3d>> flown_path_;
  std::vector<std::pair<ros::Duration, Eigen::Vector3d>> desired_path_;

  visualization_msgs::MarkerArray marker_array_;

  double trajectory_start_distance_;
  double trajectory_end_distance_;

  double min_rms_error_;
  ros::Duration time_offset_;
  ros::Duration min_time_needed_;

  void updateMarkerPath(size_t num_trajectory_points);

  std::pair<double, size_t> calcSquaredError() const;
};
}

#endif
