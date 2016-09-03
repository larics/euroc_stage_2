#ifndef TASK2_EVAL_H
#define TASK2_EVAL_H

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/time.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>
#include <mav_planning_utils/ros_trajectory_interface.h>

#include <euroc_stage2/trajectory_evaluator.h>
#include <euroc_stage2/eval_base.h>

namespace euroc_stage2 {

constexpr unsigned int kNumSubTasks = 4;
constexpr double kSubTaskLength = 25.0;
constexpr double kWindSubTaskLength = 50.0;
constexpr bool kDefaultIsWindTrajectory = false;

class Task2Eval : public EvalBase {
 public:
  
  Task2Eval(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

 private:

  void poseCallback(const geometry_msgs::TransformStampedConstPtr& msg);

  void trajectoryCallback(
    trajectory_msgs::MultiDOFJointTrajectoryConstPtr msg);

  void controlFlagUpdateCallback(const std_msgs::BoolConstPtr& msg);

  // subscribers
  ros::Subscriber pose_sub_;
  ros::Subscriber trajectory_sub_;

  // publishers
  std::vector<ros::Publisher> marker_pubs_;

  std::vector<TrajectoryEvaluator> trajectory_evaluators_;

};
}
#endif
