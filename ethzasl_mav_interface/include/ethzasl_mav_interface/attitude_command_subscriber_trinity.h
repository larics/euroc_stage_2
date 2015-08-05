/*
 * Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ATTITUDE_COMMAND_SUBSCRIBER_TRINITY_H_
#define ATTITUDE_COMMAND_SUBSCRIBER_TRINITY_H_

#include <aci/variable.h>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <ethzasl_mav_interface/common.h>

namespace ethzasl_mav_interface {

namespace trinity {

class AttitudeCommandHandler {
 public:
  AttitudeCommandHandler(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

 private:
  void attitudeThrustCallback(const mav_msgs::AttitudeThrustConstPtr& msg);
  void rollPitchYawrateThrustCallback(const mav_msgs::RollPitchYawrateThrustConstPtr& msg);

  void poseWithCovarianceStampedCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  void transformStampedCallback(const geometry_msgs::TransformStampedConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

  void fastPacketCallback();

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber attitude_thrust_subscriber_;
  ros::Subscriber roll_pitch_yaw_rate_thrust_subscriber_;

  int packet_id_;
  aci::Variable<aci::VariableQuaternion> attitude_;
  aci::Variable<aci::VariableFloat> thrust_;
  aci::Variable<aci::VariableFloat> yaw_rate_;
  aci::Variable<aci::VariableUint8> update_;
  aci::Variable<aci::VariableUint32> flags_;

  aci::Subscription fast_packet_sub_;
  aci::Variable<aci::VariableQuaternion> imu_attitude_;
  Eigen::Quaterniond current_imu_attitude_;

  ros::Subscriber pose_with_covariance_subscriber_;
  ros::Subscriber transform_stamped_subscriber_;
  ros::Subscriber odometry_subscriber_;
  Eigen::Quaterniond current_estimated_attitude_;
  ros::Time last_pose_update_time_;

  double thrust_constant_;
};

} // end namespace trinity

} // end namespace ethzasl_mav_interface

#endif /* ATTITUDE_COMMAND_SUBSCRIBER_TRINITY_H_ */
