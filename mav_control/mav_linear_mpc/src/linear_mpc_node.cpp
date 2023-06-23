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

#include <mav_control_interface/mav_control_interface.h>
#include <mav_control_interface/rc_interface_aci.h>

#include <mav_linear_mpc/linear_mpc_node.h>

namespace mav_control {

LinearModelPredictiveControllerNode::LinearModelPredictiveControllerNode(ros::NodeHandle& nh,
                                                                         ros::NodeHandle& private_nh)
    : linear_mpc_(nh, private_nh),
      sampling_time_(0.01),
      initialized_params_(false),
      dyn_config_server_(ros::NodeHandle(private_nh, "controller"))
{
  dynamic_reconfigure::Server<mav_linear_mpc::LinearMPCConfig>::CallbackType f;
  f = boost::bind(&LinearModelPredictiveControllerNode::DynConfigCallback, this, _1, _2);
  dyn_config_server_.setCallback(f);

  initialized_params_ = true;
}

LinearModelPredictiveControllerNode::~LinearModelPredictiveControllerNode()
{

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

bool LinearModelPredictiveControllerNode::setReference(
    const mav_msgs::EigenTrajectoryPoint& reference)
{
  linear_mpc_.SetCommandTrajectoryPoint(reference);
  return true;
}

bool LinearModelPredictiveControllerNode::setReferenceArray(
    const mav_msgs::EigenTrajectoryPointDeque& reference_array)
{
  linear_mpc_.SetCommandTrajectory(reference_array);
  return true;
}

bool LinearModelPredictiveControllerNode::setOdometry(const mav_msgs::EigenOdometry& odometry)
{
  linear_mpc_.SetOdometry(odometry);
  return true;
}

bool LinearModelPredictiveControllerNode::calculateRollPitchYawrateThrustCommand(
    mav_msgs::EigenRollPitchYawrateThrust* attitude_thrust_command)
{
  Eigen::Vector4d rpy_thrust;
  linear_mpc_.CalculateAttitudeThrust(&rpy_thrust);
  attitude_thrust_command->roll = rpy_thrust(0);
  attitude_thrust_command->pitch = rpy_thrust(1);
  attitude_thrust_command->yaw_rate = rpy_thrust(2);
  attitude_thrust_command->thrust.z() = rpy_thrust(3);
  std::cout << attitude_thrust_command;
  return true;
}

bool LinearModelPredictiveControllerNode::calculateAttitudeThrustCommand(
    mav_msgs::EigenAttitudeThrust* attitude_thrust_command)
{
  ROS_WARN("calculateAttitudeThrustCommand not implemented");
  return false;
}

bool LinearModelPredictiveControllerNode::getCurrentReference(mav_msgs::EigenTrajectoryPoint* reference) const
{
  assert(reference != nullptr);
  return linear_mpc_.GetCurrentReference(reference);
}

} // end namespace mav_control

int main(int argc, char** argv)
{
  ros::init(argc, argv, "LinearModelPredictiveControllerNode");

  ros::NodeHandle nh, private_nh("~");

  std::shared_ptr<mav_control::LinearModelPredictiveControllerNode> mpc(
      new mav_control::LinearModelPredictiveControllerNode(nh, private_nh));

  std::shared_ptr<mav_control_interface::RcInterfaceAci> rc(new mav_control_interface::RcInterfaceAci(nh));

  mav_control_interface::MavControlInterface control_interface(nh, private_nh, mpc, rc);

  ros::spin();

  return 0;
}
