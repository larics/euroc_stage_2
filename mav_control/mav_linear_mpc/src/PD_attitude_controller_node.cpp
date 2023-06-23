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
#include <iostream>

#include <mav_linear_mpc/PD_attitude_controller_node.h>

namespace mav_control{

PDAttitudeControllerNode::PDAttitudeControllerNode()
    :initialized_params_(false),
     got_first_attitude_command_(false){

  InitializeParams();
  PD_attitude_controller_.InitializeParams();
  ros::NodeHandle nh(namespace_);

  command_roll_pitch_yawrate_thrust_sub_ = nh.subscribe(
      mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1,
      &PDAttitudeControllerNode::CommandRollPitchYawRateThrustCallback, this);

  odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &PDAttitudeControllerNode::OdometryCallback, this);

  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  dynamic_reconfigure::Server<mav_linear_mpc::PDAttitudeConfig>::CallbackType f;
  f = boost::bind(&PDAttitudeControllerNode::DynConfigCallback, this, _1, _2);
  dyn_config_server_.setCallback(f);
}

PDAttitudeControllerNode::~PDAttitudeControllerNode() { }




void PDAttitudeControllerNode::InitializeParams(){

  ros::NodeHandle pnh("~");
  pnh.param<std::string>("robotNamespace", namespace_, "");
}


void PDAttitudeControllerNode::CommandRollPitchYawRateThrustCallback(
    const mav_msgs::RollPitchYawrateThrustConstPtr& roll_pitch_yawrate_thrust_reference){

  PD_attitude_controller_.SetDesiredAttitude(roll_pitch_yawrate_thrust_reference->roll,
                                             roll_pitch_yawrate_thrust_reference->pitch,
                                             roll_pitch_yawrate_thrust_reference->yaw_rate,
                                             roll_pitch_yawrate_thrust_reference->thrust.z);
  got_first_attitude_command_ = true;
}


void PDAttitudeControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
  ROS_INFO_ONCE("PDAttitudeController got first odometry message.");

  if(!got_first_attitude_command_)
      return;


  mav_msgs::EigenOdometry odometry;
  eigenOdometryFromMsg(*odometry_msg, &odometry);


  PD_attitude_controller_.SetOdometry(odometry);



   Eigen::VectorXd ref_rotor_velocities;
   PD_attitude_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  mav_msgs::Actuators turning_velocities_msg;

  turning_velocities_msg.angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    turning_velocities_msg.angular_velocities.push_back(ref_rotor_velocities[i]);
  turning_velocities_msg.header.stamp = odometry_msg->header.stamp;
  /*std::cout << turning_velocities_msg.angular_velocities;*/
  motor_velocity_reference_pub_.publish(turning_velocities_msg);
}


void PDAttitudeControllerNode::DynConfigCallback(mav_linear_mpc::PDAttitudeConfig &config, uint32_t level) {

  PD_attitude_controller_.SetPDParameters(config.roll_gain, config.pitch_gain, config.p_gain,
                                          config.q_gain,config.r_gain );
}

}


int main(int argc, char** argv) {
  ros::init(argc, argv, "PDAttitudeControllerNode");

  mav_control::PDAttitudeControllerNode PD_attitude_controller;

  ros::spin();

  return 0;
}
