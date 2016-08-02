#ifndef EUROC_TASK4_TASK4_EVAL_H
#define EUROC_TASK4_TASK4_EVAL_H

#include <stdio.h>
#include <Eigen/Eigen>
#include <iostream>
#include <string>

// ros
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/conversions.h>
#include <ros/ros.h>

#include <mav_msgs/default_topics.h>
#include <mav_msgs/conversions.h>
#include <euroc_stage2/eval_base.h>

namespace euroc_stage2 {

class Task4Eval : public EvalBase {
 public:
  // Max distance away from the octomap for which the EDT is calculated.
  static constexpr double kOctomapMaxDistMeters = 3.0;
  static constexpr char kOutputSeparator[] = ", ";
  // Stop recording after it's been a second from the last reference.
  static constexpr double kReferenceTimeoutSec = 1.0;

  Task4Eval(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~Task4Eval();

 private:
  // Callbacks.
  void odometryCallback(const nav_msgs::Odometry& msg);
  void twistCallback(const geometry_msgs::TwistStamped& msg);
  void setOctomapFromMsg(const octomap_msgs::Octomap& msg);

  // Setup for the scoring (calculates distance map from the loaded octomap).
  void loadOctomapFromFile();
  void initializeDistanceMap();

  // Functions to actually do the scoring...
  void calculateScoreAndOutput(const mav_msgs::EigenOdometry& odom);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // subscribers
  ros::Subscriber odometry_sub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber octomap_sub_;

  // Parameters
  double dt_sec_;
  // Determine how to clamp off the ceiling and ground of the octomap.
  double octomap_min_z_;
  double octomap_max_z_;

  // Map data.
  std::string octomap_file_;
  std::shared_ptr<octomap::OcTree> octree_;
  std::shared_ptr<DynamicEDTOctomap> distance_map_;

  // Throttle input data.
  ros::Time last_msg_time_;
  ros::Time last_ref_time_;
  bool received_twist_ref_;

  // Cached commands.
  Eigen::Vector3d last_vel_command_;
  double last_yaw_rate_command_;
};

}  // namespace euroc_stage2

#endif  // EUROC_TASK4_TASK4_EVAL_H
