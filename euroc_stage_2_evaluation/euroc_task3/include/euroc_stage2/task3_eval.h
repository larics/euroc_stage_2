#ifndef TASK3_EVAL_H
#define TASK3_EVAL_H

#include <stdio.h>
#include <Eigen/Eigen>
#include <iostream>
#include <string>

// ros
#include <ros/ros.h>
#include <euroc_stage2/eval_base.h>
#include <euroc_stage2/waypoint.h>

#include <mav_msgs/default_topics.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

namespace euroc_stage2 {

class Task3Eval : public EvalBase {
 public:
  static constexpr double kWaypointRadius =
      0.35;  // Radius in which the MAV has to stay.
  static constexpr double kWaypointHoldTime =
      10.0;  // Time in seconds to hold the waypoint.

  Task3Eval(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

 private:
  void poseCallback(const geometry_msgs::TransformStampedConstPtr& msg);
  void waypointCallback(const geometry_msgs::PoseStamped& msg);

  // subscribers
  ros::Subscriber transform_sub_;
  ros::Subscriber waypoint_sub_;

  std::vector<Waypoint> waypoint_;
};
}
#endif
