#ifndef TASK1_EVAL_H
#define TASK1_EVAL_H

#include <stdio.h>
#include <Eigen/Eigen>
#include <fstream>
#include <iostream>

// ros
#include <ros/callback_queue.h>
#include <ros/ros.h>

// ros msgs
#include <mav_msgs/default_topics.h>
#include <std_msgs/Bool.h>

// ros srvs
#include <std_srvs/Empty.h>

#include <euroc_stage2/waypoint.h>
#include <euroc_stage2/eval_base.h>

namespace euroc_stage2 {

class Task1Eval : public EvalBase {
 public:
  Task1Eval(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

 private:
  Waypoint readWaypointParams();

  bool startTaskEvaluationCallback(std_srvs::EmptyRequest& request,
                                   std_srvs::EmptyResponse& response);
  void viconUpdateCallback(const geometry_msgs::TransformStampedConstPtr& msg);

  // service
  ros::ServiceServer start_srv_;

  // subscribers
  ros::Subscriber vicon_sub_;

  // publishers
  ros::Publisher vis_pub_;

  size_t waypoint_idx_;
  std::shared_ptr<Waypoint> waypoint_;

  bool finished_;
};

// Default values
constexpr double kSmallTime = 0.0001;
constexpr double kDefaultWaypointPositionX = 0.0;
constexpr double kDefaultWaypointPositionY = 0.0;
constexpr double kDefaultWaypointPositionZ = 0.0;
constexpr double kDefaultWaypointRadius = 0.0;
constexpr double kDefaultWaypointHoldTime = 0.0;
}
#endif
