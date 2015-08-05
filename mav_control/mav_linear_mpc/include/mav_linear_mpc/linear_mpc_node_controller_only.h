/*
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 *
 * This code can be used only for academic, non-commercial use.
 * This code cannot be redistributed under any license, open source or otherwise.
 *
 * CVXGEN license: http://cvxgen.com/docs/license.html
 * FORCES license: http://forces.ethz.ch
 *
 */

#ifndef LINEAR_MPC_NODE_CONTROLLER_ONLY_H
#define LINEAR_MPC_NODE_CONTROLLER_ONLY_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

//ros
#include <ros/ros.h>
#include <ros/callback_queue.h>

//ros msgs
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_linear_mpc/common.h>
#include <mav_linear_mpc/linear_mpc.h>

//dynamic reconfiguration
#include <dynamic_reconfigure/server.h>
#include <mav_linear_mpc/LinearMPCConfig.h>

namespace mav_control {

class LinearModelPredictiveControllerNode
{
 public:
  LinearModelPredictiveControllerNode(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  ~LinearModelPredictiveControllerNode();

  void InitializeParams();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:

  LinearModelPredictiveController linear_mpc_;
  const double sampling_time_;
  bool initialized_params_;
  bool got_first_trajectory_command_;

  //subscribers
  ros::Subscriber odometry_sub_;
  ros::Subscriber cmd_trajectory_sub_;
  ros::Subscriber cmd_pose_sub_;

  //publishers
  ros::Publisher command_roll_pitch_yawrate_thrust_pub_;

  mav_msgs::EigenTrajectoryPointDeque command_trajectory_position_yaw_array_;
  dynamic_reconfigure::Server<mav_linear_mpc::LinearMPCConfig> dyn_config_server_;

  void CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  void CommandTrajectoryCallback(
      const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_array_msg);

  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

  void DynConfigCallback(mav_linear_mpc::LinearMPCConfig &config, uint32_t level);
};

}
#endif
