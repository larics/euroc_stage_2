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

#include <aci/aci.h>
#include <aci/variable_defines_trinity.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mav_msgs/default_topics.h>

#include <ethzasl_mav_interface/attitude_command_subscriber_trinity.h>
#include <ethzasl_mav_interface/common.h>
#include <ethzasl_mav_interface/helper.h>

namespace ethzasl_mav_interface {

namespace trinity {

AttitudeCommandHandler::AttitudeCommandHandler(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      packet_id_(-1),
      current_imu_attitude_(Eigen::Quaterniond::Identity()),
      current_estimated_attitude_(Eigen::Quaterniond::Identity()),
      last_pose_update_time_(0),
      thrust_constant_(1.0) {

  aci::Aci& aci = aci::instance();

  packet_id_ = aci.getNextFreeCommandPacketId();
  if (packet_id_ == -1) {
    ROS_ERROR_STREAM("Trinity attitude command subscriber: no more command packets available.");
    return;
  }
  else {
    ROS_INFO_STREAM("Trinity attitude command subscriber: using packet_id " << packet_id_);
  }

  attitude_ = aci.registerCommand<aci::VariableQuaternion>(packet_id_, ACI_USER_CMD_ATTITUDE_QUAT);
  thrust_ = aci.registerCommand<aci::VariableFloat>(packet_id_, ACI_USER_CMD_ATTITUDE_THRUST);
  yaw_rate_ = aci.registerCommand<aci::VariableFloat>(packet_id_, ACI_USER_CMD_ATTITUDE_YAW_RATE);

  update_ = aci.registerCommand<aci::VariableUint8>(packet_id_, ACI_USER_CMD_ATTITUDE_UPDATE);
  flags_ = aci.registerCommand<aci::VariableUint32>(packet_id_, ACI_USER_CMD_ATTITUDE_FLAGS);

  aci.updateCommandConfiguration(packet_id_, false);

  attitude_thrust_subscriber_ = nh.subscribe(mav_msgs::default_topics::COMMAND_ATTITUDE_THRUST, 1,
                                             &AttitudeCommandHandler::attitudeThrustCallback, this,
                                             ros::TransportHints().tcpNoDelay());

  roll_pitch_yaw_rate_thrust_subscriber_ = nh.subscribe(
      mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 1,
      &AttitudeCommandHandler::rollPitchYawrateThrustCallback, this, ros::TransportHints().tcpNoDelay());

  // Listen to imu attitude, in order to transform the absolute yaw command correctly.
  imu_attitude_ = aci.registerVariable<aci::VariableQuaternion>(aci::VAR_PACKET_FAST,
                                                                ACI_USER_VAR_IMU_ATTITUDE_QUAT);

  fast_packet_sub_ = aci.registerVariableCallback(aci::VAR_PACKET_FAST,
                                                  &AttitudeCommandHandler::fastPacketCallback, this);

  aci.updateVariableConfiguration(aci::VAR_PACKET_FAST);

  pose_with_covariance_subscriber_ = nh.subscribe(mav_msgs::default_topics::POSE_WITH_COVARIANCE, 1,
                                                  &AttitudeCommandHandler::poseWithCovarianceStampedCallback,
                                                  this);

  transform_stamped_subscriber_ = nh.subscribe(mav_msgs::default_topics::TRANSFORM, 1,
                                               &AttitudeCommandHandler::transformStampedCallback, this);

  odometry_subscriber_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                      &AttitudeCommandHandler::odometryCallback, this);

  private_nh_.param("thrust_constant", thrust_constant_, 1.0);
}

void AttitudeCommandHandler::attitudeThrustCallback(const mav_msgs::AttitudeThrustConstPtr& msg) {

  Eigen::Quaterniond q_ros(msg->attitude.w, msg->attitude.x, msg->attitude.y, msg->attitude.z);

  update_ = msg->header.seq % (0x100);
  flags_ = EXT_ATTITUDE_CTRL_ACTIVE | EXT_ATTITUDE_THRUST_ENABLED | EXT_ATTITUDE_QUATERNION_ENABLED;

// TODO(acmarkus): implement this
//  // q_est = q_imu * q_err
////  const Eigen::Quaterniond q_err = current_imu_attitude_.inverse() * current_estimated_attitude_;
////  const double yaw_err = helper::yawFromQuaternion(q_err);
//
//  const Eigen::Quaterniond q_asctec = helper::rosQuaternionToAsctec(q_ros);
//
//  attitude_.getPtr()->w = q_asctec.w();
//  attitude_.getPtr()->x = q_asctec.x();
//  attitude_.getPtr()->y = q_asctec.y();
//  attitude_.getPtr()->z = q_asctec.z();
//
//  thrust_ = msg->thrust;  // TODO(acmarkus): FIX THIS!!!
//
//  aci::instance().sendCommandPacketToDevice(packet_id_);
//
//  ROS_INFO_STREAM(
//      "sent control: q=" << attitude_.value().w << " "<< attitude_.value().x << " "<< attitude_.value().y <<
//      " "<< attitude_.value().z << " " << " thrust:" << msg->thrust);
}

void AttitudeCommandHandler::rollPitchYawrateThrustCallback(
    const mav_msgs::RollPitchYawrateThrustConstPtr& msg) {

  update_ = msg->header.seq % 0x100;
  flags_ = EXT_ATTITUDE_CTRL_ACTIVE | EXT_ATTITUDE_YAW_RATE_ENABLED | EXT_ATTITUDE_THRUST_ENABLED
      | EXT_ATTITUDE_QUATERNION_ENABLED;

  const Eigen::Quaterniond q_cmd = helper::rpyToQuaternion<double>(msg->roll, -msg->pitch, 0.0);

  attitude_.getPtr()->w = q_cmd.w();
  attitude_.getPtr()->x = q_cmd.x();
  attitude_.getPtr()->y = q_cmd.y();
  attitude_.getPtr()->z = q_cmd.z();

  // Asctec uses NED, thus we turn into the opposite direction.
  yaw_rate_ = -msg->yaw_rate;

  thrust_ = msg->thrust.z * thrust_constant_;  // TODO(asctec): thrust should be commanded as force, not acceleration @ weight ??.

  aci::instance().sendCommandPacketToDevice(packet_id_);
}

void AttitudeCommandHandler::fastPacketCallback() {
  quaternion _q_asctec = attitude_.value();
  Eigen::Quaterniond q_asctec(_q_asctec.w, _q_asctec.x, _q_asctec.y, _q_asctec.z);
  current_imu_attitude_ = helper::asctecQuaternionToRos(q_asctec);
}

void AttitudeCommandHandler::poseWithCovarianceStampedCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
  current_estimated_attitude_.w() = msg->pose.pose.orientation.w;
  current_estimated_attitude_.x() = msg->pose.pose.orientation.x;
  current_estimated_attitude_.y() = msg->pose.pose.orientation.y;
  current_estimated_attitude_.z() = msg->pose.pose.orientation.z;
  last_pose_update_time_ = ros::Time::now();
}

void AttitudeCommandHandler::transformStampedCallback(const geometry_msgs::TransformStampedConstPtr& msg) {
  current_estimated_attitude_.w() = msg->transform.rotation.w;
  current_estimated_attitude_.x() = msg->transform.rotation.x;
  current_estimated_attitude_.y() = msg->transform.rotation.y;
  current_estimated_attitude_.z() = msg->transform.rotation.z;
  last_pose_update_time_ = ros::Time::now();
}

void AttitudeCommandHandler::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  current_estimated_attitude_.w() = msg->pose.pose.orientation.w;
  current_estimated_attitude_.x() = msg->pose.pose.orientation.x;
  current_estimated_attitude_.y() = msg->pose.pose.orientation.y;
  current_estimated_attitude_.z() = msg->pose.pose.orientation.z;
  last_pose_update_time_ = ros::Time::now();
}

}  // end namespace trinity

}  // end namespace ethzasl_mav_interface
