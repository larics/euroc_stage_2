/*
* Copyright (c) 2014, Markus Achtelik, ASL, ETH Zurich, Switzerland
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



#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/Actuators.h>

// #include <ethzasl_mav_firmware/aci_variable_names.h>
#include <aci/variable_defines_autopilot.h>
#include <ethzasl_mav_interface/common.h>
#include <ethzasl_mav_interface/common_data_publisher_autopilot.h>
#include <ethzasl_mav_interface/helper.h>

namespace ethzasl_mav_interface {

namespace autopilot {

constexpr int CommonDataPublisher::kNumberOfRotors;

CommonDataPublisher::CommonDataPublisher(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh) {

  private_nh_.param("frame_id", frame_id_, std::string("fcu"));
  double angular_velocity_standard_deviation;
  double linear_acceleration_standard_deviation;
  private_nh_.param("angular_velocity_standard_deviation", angular_velocity_standard_deviation, 0.013);  // taken from experiments
  private_nh_.param("linear_acceleration_standard_deviation", linear_acceleration_standard_deviation, 0.083);  // taken from experiments
  angular_velocity_variance_ = angular_velocity_standard_deviation * angular_velocity_standard_deviation;
  linear_acceleration_variance_ = linear_acceleration_standard_deviation * linear_acceleration_standard_deviation;

  imu_pub_ = nh_.advertise<sensor_msgs::Imu>(mav_msgs::default_topics::IMU, 1);
  motors_pub_ = nh_.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::MOTOR_MEASUREMENT, 1);
  gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(mav_msgs::default_topics::GPS, 1);
//    rc_pub_ = nh_.advertise<asctec_hl_comm::mav_rcdata> ("rcdata", 1);
//    status_pub_ = nh_.advertise<asctec_hl_comm::mav_status> ("status", 1);
//  mag_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("mag", 1);

  aci::Aci& aci = aci::instance();

  fcu_time_ = aci.registerVariable<aci::VariableInt64>(aci::VAR_PACKET_FAST, ACI_HL_UP_TIME_MS);

  angle_roll_ = aci.registerVariable<aci::VariableInt32>(aci::VAR_PACKET_FAST, ACI_ANGLE_ROLL);
  angle_pitch_ = aci.registerVariable<aci::VariableInt32>(aci::VAR_PACKET_FAST, ACI_ANGLE_PITCH);
  angle_yaw_ = aci.registerVariable<aci::VariableInt32>(aci::VAR_PACKET_FAST, ACI_ANGLE_YAW);

  angvel_roll_ = aci.registerVariable<aci::VariableInt32>(aci::VAR_PACKET_FAST, ACI_ANGVEL_ROLL);
  angvel_pitch_ = aci.registerVariable<aci::VariableInt32>(aci::VAR_PACKET_FAST, ACI_ANGVEL_PITCH);
  angvel_yaw_ = aci.registerVariable<aci::VariableInt32>(aci::VAR_PACKET_FAST, ACI_ANGVEL_YAW);

  acc_x_ = aci.registerVariable<aci::VariableInt16>(aci::VAR_PACKET_FAST, ACI_ACC_X);
  acc_y_ = aci.registerVariable<aci::VariableInt16>(aci::VAR_PACKET_FAST, ACI_ACC_Y);
  acc_z_ = aci.registerVariable<aci::VariableInt16>(aci::VAR_PACKET_FAST, ACI_ACC_Z);

  motor_speed_[0] = aci.registerVariable<aci::VariableUint8>(aci::VAR_PACKET_FAST, ACI_MOTOR_RPM0);
  motor_speed_[1] = aci.registerVariable<aci::VariableUint8>(aci::VAR_PACKET_FAST, ACI_MOTOR_RPM1);
  motor_speed_[2] = aci.registerVariable<aci::VariableUint8>(aci::VAR_PACKET_FAST, ACI_MOTOR_RPM2);
  motor_speed_[3] = aci.registerVariable<aci::VariableUint8>(aci::VAR_PACKET_FAST, ACI_MOTOR_RPM3);
  motor_speed_[4] = aci.registerVariable<aci::VariableUint8>(aci::VAR_PACKET_FAST, ACI_MOTOR_RPM4);
  motor_speed_[5] = aci.registerVariable<aci::VariableUint8>(aci::VAR_PACKET_FAST, ACI_MOTOR_RPM5);

  aci.updateVariableConfiguration(aci::VAR_PACKET_FAST);
  fast_packet_sub_ = aci.registerVariableCallback(aci::VAR_PACKET_FAST, &CommonDataPublisher::fastPacketCallback, this);
}

void CommonDataPublisher::fastPacketCallback(){
  std_msgs::Header header;
  header.frame_id = frame_id_;
  header.stamp = ros::Time::now(); // TODO(acmarkus): replace with proper timesync!!!
  publishImu(header);
  publishMotors(header);
}

void CommonDataPublisher::publishImu(const std_msgs::Header& header) {

  size_t n_imu_subscribers = imu_pub_.getNumSubscribers();

  if (n_imu_subscribers > 0) {
    const double roll = helper::autopilot::attitudeToSI(angle_roll_.value());
    const double pitch = helper::autopilot::attitudeToSI(angle_pitch_.value());
    double yaw = helper::autopilot::attitudeToSI(angle_yaw_.value());

    if (yaw > M_PI)
      yaw -= 2 * M_PI;

    const Eigen::Quaterniond q_asctec = helper::rpyToQuaternion(roll, pitch, yaw);
    const Eigen::Quaterniond q_ros = helper::asctecQuaternionToRos(q_asctec);

    sensor_msgs::ImuPtr msg(new sensor_msgs::Imu);
    msg->header = header;

    msg->linear_acceleration.x = helper::autopilot::accToSI(acc_x_.value());
    msg->linear_acceleration.y = -helper::autopilot::accToSI(acc_y_.value());
    msg->linear_acceleration.z = -helper::autopilot::accToSI(acc_z_.value());
    msg->angular_velocity.x = helper::autopilot::omegaToSI(angvel_roll_.value());
    msg->angular_velocity.y = -helper::autopilot::omegaToSI(angvel_pitch_.value());
    msg->angular_velocity.z = -helper::autopilot::omegaToSI(angle_yaw_.value());
    msg->orientation.w = q_ros.w();
    msg->orientation.x = q_ros.x();
    msg->orientation.y = q_ros.y();
    msg->orientation.z = q_ros.z();

    helper::setDiagonal3dCovariance(0.0, &(msg->orientation_covariance)); // covariance unknown
    helper::setDiagonal3dCovariance(angular_velocity_variance_, &(msg->angular_velocity_covariance));
    helper::setDiagonal3dCovariance(linear_acceleration_variance_, &(msg->linear_acceleration_covariance));

    imu_pub_.publish(msg);
  }
}

void CommonDataPublisher::publishMotors(const std_msgs::Header& header) {
  size_t n_motor_subscribers = motors_pub_.getNumSubscribers();

  if (n_motor_subscribers > 0) {
    mav_msgs::ActuatorsPtr msg(new mav_msgs::Actuators);
    msg->header = header;

    msg->angular_velocities.resize(kNumberOfRotors);

    for (size_t i = 0; i < kNumberOfRotors; ++i) {
      // Real motor speed [rpm] = motors * 64 (from sdk.h). Then convert to [rad/s].
      const double magic_asctec_constant = 64.0;
      msg->angular_velocities[i] = static_cast<mav_msgs::Actuators::_angular_velocities_type::value_type>(motor_speed_[i].value())
          * magic_asctec_constant * 60.0 * 2.0 * M_PI;
    }

    motors_pub_.publish(msg);
  }
}

}  // end namespace trinity

}  // end namespace ethzasl_mav_interface
