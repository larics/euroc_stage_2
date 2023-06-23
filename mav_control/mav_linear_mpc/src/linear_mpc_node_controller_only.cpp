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

#include <mav_msgs/default_topics.h>

#include <mav_linear_mpc/linear_mpc_node_controller_only.h>

namespace mav_control {

LinearModelPredictiveControllerNode::LinearModelPredictiveControllerNode(ros::NodeHandle& nh,
                                                                         ros::NodeHandle& private_nh)
    : linear_mpc_(nh, private_nh),
      sampling_time_(0.01),
      initialized_params_(false),
      got_first_trajectory_command_(false),
      dyn_config_server_(ros::NodeHandle(private_nh, "controller"))
{
  points_pose_sub_ = nh.subscribe("/points", 1,
                               &LinearModelPredictiveControllerNode::PointsPoseCallback, this,
                               ros::TransportHints().tcpNoDelay());
  
  
  cmd_pose_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_POSE, 1,
                               &LinearModelPredictiveControllerNode::CommandPoseCallback, this,
                               ros::TransportHints().tcpNoDelay());

  cmd_trajectory_sub_ = nh.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
                                     &LinearModelPredictiveControllerNode::CommandTrajectoryCallback,
                                     this, ros::TransportHints().tcpNoDelay());

  odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &LinearModelPredictiveControllerNode::OdometryCallback, this,
                               ros::TransportHints().tcpNoDelay());

  command_roll_pitch_yawrate_thrust_pub_ = nh.advertise<mav_msgs::RollPitchYawrateThrust>(
      mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1);

  dynamic_reconfigure::Server<mav_linear_mpc::LinearMPCConfig>::CallbackType f;
  f = boost::bind(&LinearModelPredictiveControllerNode::DynConfigCallback, this, _1, _2);
  dyn_config_server_.setCallback(f);

  initialized_params_ = true;
}

LinearModelPredictiveControllerNode::~LinearModelPredictiveControllerNode()
{

}

void LinearModelPredictiveControllerNode::CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*msg, &trajectory_point);

  linear_mpc_.SetCommandTrajectoryPoint(trajectory_point);
  got_first_trajectory_command_ = true;
}

void LinearModelPredictiveControllerNode::PointsPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*msg, &trajectory_point);

  linear_mpc_.SetCommandTrajectoryPoint(trajectory_point);
  got_first_trajectory_command_ = true;
}

void LinearModelPredictiveControllerNode::CommandTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_array_msg)
{

  command_trajectory_position_yaw_array_.clear();

  int array_size = trajectory_reference_array_msg->points.size();
  if (array_size == 0)
    return;

  mav_msgs::eigenTrajectoryPointDequeFromMsg(*trajectory_reference_array_msg,
                                             &command_trajectory_position_yaw_array_);

  got_first_trajectory_command_ = true;
  linear_mpc_.SetCommandTrajectory(command_trajectory_position_yaw_array_);
}

void LinearModelPredictiveControllerNode::DynConfigCallback(mav_linear_mpc::LinearMPCConfig &config,
                                                            uint32_t level)
{
  Eigen::VectorXd Q_vec(8);
  Eigen::VectorXd R_vec(3);
  Eigen::VectorXd R_delta_vec(3);

  Q_vec << config.q_x, config.q_y, config.q_z, config.q_vx, config.q_vy, config.q_vz, config.q_roll, config
      .q_pitch;

  R_vec << config.r_roll, config.r_pitch, config.r_thrust;
  R_delta_vec << config.r_droll, config.r_dpitch, config.r_dthrust;

  linear_mpc_.SetUseXyOffsetFree(config.use_xy_offset_free);
  linear_mpc_.SetUseHeightErrorIntegrator(config.use_height_error_integrator);
  linear_mpc_.SetUseKfEstimatedStates(config.use_KF_estimated_state);

  linear_mpc_.SetStatePenalityMatrix(Q_vec.asDiagonal());
  linear_mpc_.SetCommandPenalityMatrix(R_vec.asDiagonal());
  linear_mpc_.SetDeltaCommandPenalityMatrix(
      R_delta_vec.asDiagonal() * (1.0 / (sampling_time_ * sampling_time_)));
  linear_mpc_.SetMaximumCommand(config.roll_max, config.pitch_max, config.thrust_max);

  linear_mpc_.SetYawGain(config.K_yaw);
  linear_mpc_.SetHeightIntratorGain(config.Ki_height);
  //linear_mpc_.SetUseCommandFilter(config.use_command_position_filter, config.use_command_velocity_filter);
  linear_mpc_.ApplyParameters();
}

void LinearModelPredictiveControllerNode::OdometryCallback(
    const nav_msgs::OdometryConstPtr& odometry_msg)
{
  ROS_INFO_ONCE("MPC controller got first odometry message.");

  if (!got_first_trajectory_command_)
    return;

  mav_msgs::EigenOdometry odometry;
  mav_msgs::eigenOdometryFromMsg(*odometry_msg, &odometry);
  linear_mpc_.SetOdometry(odometry);

  Eigen::Vector4d ref_roll_pitch_yawrate_thrust;
  linear_mpc_.CalculateAttitudeThrust(&ref_roll_pitch_yawrate_thrust);

  mav_msgs::RollPitchYawrateThrust command_roll_pitch_yawrate_thrust_msg;
  command_roll_pitch_yawrate_thrust_msg.header = odometry_msg->header;
  command_roll_pitch_yawrate_thrust_msg.roll = ref_roll_pitch_yawrate_thrust(0);
  command_roll_pitch_yawrate_thrust_msg.pitch = ref_roll_pitch_yawrate_thrust(1);
  command_roll_pitch_yawrate_thrust_msg.yaw_rate = ref_roll_pitch_yawrate_thrust(2);
  command_roll_pitch_yawrate_thrust_msg.thrust.z = ref_roll_pitch_yawrate_thrust(3);
  command_roll_pitch_yawrate_thrust_pub_.publish(command_roll_pitch_yawrate_thrust_msg);
}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "LinearModelPredictiveControllerNode");

  ros::NodeHandle nh, private_nh("~");

  mav_control::LinearModelPredictiveControllerNode mpc_controller_node(nh, private_nh);

  ros::spin();

  return 0;
}
