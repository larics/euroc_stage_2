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
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/Status.h>

#include <aci/variable_defines_trinity.h>
#include <ethzasl_mav_interface/common.h>
#include <ethzasl_mav_interface/common_data_publisher_trinity.h>
#include <ethzasl_mav_interface/helper.h>

namespace ethzasl_mav_interface {

namespace trinity {

constexpr int CommonDataPublisher::kNumberOfRotors;

CommonDataPublisher::CommonDataPublisher(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      publish_commanded_motor_speeds_(false),
      publish_raw_imu_(false),
      publish_nav1_imu_(false),
      publisher_queue_size_(1)
{

  private_nh_.param("frame_id", frame_id_, std::string("fcu"));
  double angular_velocity_standard_deviation;
  double linear_acceleration_standard_deviation;
  private_nh_.param("angular_velocity_standard_deviation", angular_velocity_standard_deviation, 0.013); // TODO(burrimi): find out trinity value
  private_nh_.param("linear_acceleration_standard_deviation", linear_acceleration_standard_deviation, 0.083); // TODO(burrimi): find out trinity value
  angular_velocity_variance_ = angular_velocity_standard_deviation * angular_velocity_standard_deviation;
  linear_acceleration_variance_ = linear_acceleration_standard_deviation
      * linear_acceleration_standard_deviation;

  private_nh_.param("publish_commanded_motor_speeds", publish_commanded_motor_speeds_, false);
  private_nh_.param("publish_raw_imu", publish_raw_imu_, publish_raw_imu_);
  private_nh_.param("publish_nav1_imu", publish_nav1_imu_, publish_nav1_imu_);
  private_nh_.param("publisher_queue_size", publisher_queue_size_, publisher_queue_size_);

  imu_pub_ = nh_.advertise<sensor_msgs::Imu>(mav_msgs::default_topics::IMU, publisher_queue_size_);
  motors_pub_ = nh_.advertise<mav_msgs::Actuators>(mav_msgs::default_topics::MOTOR_MEASUREMENT, publisher_queue_size_);
  gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>(mav_msgs::default_topics::GPS, publisher_queue_size_);
  rc_pub_ = nh_.advertise<sensor_msgs::Joy> (mav_msgs::default_topics::RC, publisher_queue_size_);
  status_pub_ = nh_.advertise<mav_msgs::Status> (mav_msgs::default_topics::STATUS, publisher_queue_size_);
//  discharge_current_pub_ = nh_.advertise<std_msgs::Float32>(kDefaultDischargeCurrentTopic, 1); //TODO: re-enable, once fully supported.
  safety_pilot_status_pub_ = nh_.advertise<std_msgs::String>(kDefaultSafetyPilotStatusTopic, publisher_queue_size_);

  if (publish_commanded_motor_speeds_) {
    motors_cmd_pub_ = nh_.advertise<mav_msgs::Actuators>(
        std::string(mav_msgs::default_topics::MOTOR_MEASUREMENT) + "_commanded", publisher_queue_size_);
    ROS_INFO("Will publish commanded motor speeds.");
  } else {
    ROS_INFO("Will NOT publish commanded motor speeds. "
             "To change, set \"~/publish_commanded_motor_speeds\" to true.");
  }

  if (publish_raw_imu_) {
    ROS_INFO("Will publish raw IMU measurements");
    imu_pub_raw_ = nh_.advertise<sensor_msgs::Imu>(std::string(mav_msgs::default_topics::IMU) + "_raw", publisher_queue_size_);
  }
  else {
    ROS_INFO("Will NOT publish raw IMU measurements"
             "To change, set \"~/publish_raw_imu\" to true.");
  }

  if (publish_nav1_imu_) {
    ROS_INFO("Will publish nav1 IMU measurements");
    imu_pub_nav1_ = nh_.advertise<sensor_msgs::Imu>(std::string(mav_msgs::default_topics::IMU) + "_nav1", publisher_queue_size_);
  }
  else {
    ROS_INFO("Will NOT publish nav1 IMU measurements"
             "To change, set \"~/publish_nav1_imu\" to true.");
  }

//  mag_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("mag", 1);

  aci::Aci& aci = aci::instance();

  fcu_time_us_ = aci.registerVariable<aci::VariableInt64>(aci::VAR_PACKET_FAST, ACI_USER_VAR_USCOUNTER);

  attitude_ = aci.registerVariable<aci::VariableQuaternion>(aci::VAR_PACKET_FAST,
                                                            ACI_USER_VAR_IMU_ATTITUDE_QUAT);

  angular_velocity_ = aci.registerVariable<aci::VariableVector3f>(aci::VAR_PACKET_FAST, ACI_USER_VAR_IMU_W);
  acceleration_ = aci.registerVariable<aci::VariableVector3f>(aci::VAR_PACKET_FAST, ACI_USER_VAR_IMU_ACC);

  if (publish_raw_imu_) {
    angular_velocity_raw_ = aci.registerVariable<aci::VariableVector3i>(aci::VAR_PACKET_FAST,
                                                                        ACI_USER_VAR_IMU_RAW_GYRO);
    acceleration_raw_ = aci.registerVariable<aci::VariableVector3i>(aci::VAR_PACKET_FAST,
                                                                    ACI_USER_VAR_IMU_RAW_ACC);
  }

  if (publish_nav1_imu_) {
    angular_velocity_nav1_ = aci.registerVariable<aci::VariableVector3f>(aci::VAR_PACKET_FAST,
                                                                         ACI_USER_VAR_IMU_W_NAV1);
    acceleration_nav1_ = aci.registerVariable<aci::VariableVector3f>(aci::VAR_PACKET_FAST,
                                                                     ACI_USER_VAR_IMU_ACC_NAV1);
  }

  motor_speed_measured_[0] = aci.registerVariable<aci::VariableUint16>(aci::VAR_PACKET_FAST,
                                                                       ACI_USER_VAR_MOTOR0_RPM);
  motor_speed_measured_[1] = aci.registerVariable<aci::VariableUint16>(aci::VAR_PACKET_FAST,
                                                                       ACI_USER_VAR_MOTOR1_RPM);
  motor_speed_measured_[2] = aci.registerVariable<aci::VariableUint16>(aci::VAR_PACKET_FAST,
                                                                       ACI_USER_VAR_MOTOR2_RPM);
  motor_speed_measured_[3] = aci.registerVariable<aci::VariableUint16>(aci::VAR_PACKET_FAST,
                                                                       ACI_USER_VAR_MOTOR3_RPM);
  motor_speed_measured_[4] = aci.registerVariable<aci::VariableUint16>(aci::VAR_PACKET_FAST,
                                                                       ACI_USER_VAR_MOTOR4_RPM);
  motor_speed_measured_[5] = aci.registerVariable<aci::VariableUint16>(aci::VAR_PACKET_FAST,
                                                                       ACI_USER_VAR_MOTOR5_RPM);
  if (publish_commanded_motor_speeds_) {
    motor_speed_commanded_[0] = aci.registerVariable<aci::VariableUint16>(aci::VAR_PACKET_FAST,
                                                                          ACI_USER_VAR_MOTOR0_CMD);
    motor_speed_commanded_[1] = aci.registerVariable<aci::VariableUint16>(aci::VAR_PACKET_FAST,
                                                                          ACI_USER_VAR_MOTOR1_CMD);
    motor_speed_commanded_[2] = aci.registerVariable<aci::VariableUint16>(aci::VAR_PACKET_FAST,
                                                                          ACI_USER_VAR_MOTOR2_CMD);
    motor_speed_commanded_[3] = aci.registerVariable<aci::VariableUint16>(aci::VAR_PACKET_FAST,
                                                                          ACI_USER_VAR_MOTOR3_CMD);
    motor_speed_commanded_[4] = aci.registerVariable<aci::VariableUint16>(aci::VAR_PACKET_FAST,
                                                                          ACI_USER_VAR_MOTOR4_CMD);
    motor_speed_commanded_[5] = aci.registerVariable<aci::VariableUint16>(aci::VAR_PACKET_FAST,
                                                                          ACI_USER_VAR_MOTOR5_CMD);
  }

  discharge_current_ = aci.registerVariable<aci::VariableUint16>(aci::VAR_PACKET_FAST, ACI_USER_VAR_CURRENT);

  rc_data_lock_ = aci.registerVariable<aci::VariableUint8>(aci::VAR_PACKET_MEDIUM, ACI_USER_VAR_RCDATA_LOCK);
  rc_roll_ = aci.registerVariable<aci::VariableInt16>(aci::VAR_PACKET_MEDIUM,
                                                      ACI_USER_VAR_RCDATA_CHANNEL_ROLL);
  rc_pitch_ = aci.registerVariable<aci::VariableInt16>(aci::VAR_PACKET_MEDIUM,
                                                       ACI_USER_VAR_RCDATA_CHANNEL_PITCH);
  rc_yaw_ = aci.registerVariable<aci::VariableInt16>(aci::VAR_PACKET_MEDIUM, ACI_USER_VAR_RCDATA_CHANNEL_YAW);
  rc_thrust_ = aci.registerVariable<aci::VariableInt16>(aci::VAR_PACKET_MEDIUM,
                                                        ACI_USER_VAR_RCDATA_CHANNEL_THRUST);
  rc_mode_ = aci.registerVariable<aci::VariableInt16>(aci::VAR_PACKET_MEDIUM,
                                                      ACI_USER_VAR_RCDATA_CHANNEL_MODE);
  rc_power_on_off_ = aci.registerVariable<aci::VariableInt16>(aci::VAR_PACKET_MEDIUM,
                                                              ACI_USER_VAR_RCDATA_CHANNEL_POWERONOFF);
  rc_safety_pilot_switch_ = aci.registerVariable<aci::VariableInt16>(
      aci::VAR_PACKET_MEDIUM, ACI_USER_VAR_RCDATA_CHANNEL_SAFETYPILOTSWITCH);
  rc_external_command_switch_ = aci.registerVariable<aci::VariableInt16>(
      aci::VAR_PACKET_MEDIUM, ACI_USER_VAR_RCDATA_CHANNEL_EXTCMDSWITCH);

  flight_time_ = aci.registerVariable<aci::VariableUint32>(aci::VAR_PACKET_SLOW, ACI_USER_VAR_FLIGHTTIME);
  flight_mode_ = aci.registerVariable<aci::VariableUint32>(aci::VAR_PACKET_SLOW, ACI_USER_VAR_FLIGHTMODE);
  command_status_ = aci.registerVariable<aci::VariableUint8>(aci::VAR_PACKET_SLOW, ACI_USER_VAR_CMD_STATUS);
  gps_satellites_ = aci.registerVariable<aci::VariableUint32>(aci::VAR_PACKET_SLOW, ACI_USER_VAR_GPS_SAT_NUM);
  gps_status_ = aci.registerVariable<aci::VariableInt32>(aci::VAR_PACKET_SLOW, ACI_USER_VAR_GPS_STATUS);
  cpu_time_ = aci.registerVariable<aci::VariableUint16>(aci::VAR_PACKET_SLOW, ACI_USER_VAR_CPU_TIME);
  battery_voltage_ = aci.registerVariable<aci::VariableUint16>(aci::VAR_PACKET_SLOW, ACI_USER_VAR_VOLTAGE);
  safety_pilot_status_ = aci.registerVariable<aci::VariableUint8>(aci::VAR_PACKET_SLOW,
                                                                  ACI_USER_VAR_SAFETY_PILOT_STATE);

  aci.updateVariableConfiguration(aci::VAR_PACKET_FAST);
  aci.updateVariableConfiguration(aci::VAR_PACKET_MEDIUM);
  aci.updateVariableConfiguration(aci::VAR_PACKET_SLOW);

  aci_fast_packet_sub_ = aci.registerVariableCallback(aci::VAR_PACKET_FAST,
                                                      &CommonDataPublisher::fastPacketCallback, this);
  aci_medium_packet_sub_ = aci.registerVariableCallback(aci::VAR_PACKET_MEDIUM,
                                                        &CommonDataPublisher::mediumPacketCallback, this);
  aci_slow_packet_sub_ = aci.registerVariableCallback(aci::VAR_PACKET_SLOW,
                                                      &CommonDataPublisher::slowPacketCallback, this);
}

bool CommonDataPublisher::createHeaderFromFcuTime(int64_t fcu_time_us, std_msgs::Header* header) {
  if (header == nullptr)
    return false;

  const double trinity_time = helper::trinity::convertTime(fcu_time_us);
  double local_time;
  bool success = aci::instance().deviceTimeToLocalTime(trinity_time, &local_time);
  if (!success) {
    return false;
  }

  header->frame_id = frame_id_;
  //  header.stamp = ros::Time::now(); // hopefully not necessary anymore :)

  header->stamp = ros::Time(local_time);
  return true;
}

void CommonDataPublisher::fastPacketCallback() {
  std_msgs::Header header;
  if (createHeaderFromFcuTime(fcu_time_us_.value(), &header)) {
    publishImu(header);
    publishMotors(header);
//    publishDischargeCurrent(header); //TODO: re-enable, once fully supported.
  }
  else
    ROS_ERROR_STREAM("time conversion from trinity failed. Not publishing");
}

void CommonDataPublisher::mediumPacketCallback() {
  std_msgs::Header header;
  if (createHeaderFromFcuTime(fcu_time_us_.value(), &header)) {
    publishRc(header);
  }
  else
    ROS_ERROR_STREAM("time conversion from trinity failed. Not publishing");
}

void CommonDataPublisher::slowPacketCallback() {
  std_msgs::Header header;
  if (createHeaderFromFcuTime(fcu_time_us_.value(), &header)) {
    publishStatus(header);
  }
  else
    ROS_ERROR_STREAM("time conversion from trinity failed. Not publishing");
}

void CommonDataPublisher::publishImu(const std_msgs::Header& header) {

  const size_t n_imu_subscribers = imu_pub_.getNumSubscribers();

  if (n_imu_subscribers > 0) {
    sensor_msgs::ImuPtr msg(new sensor_msgs::Imu);
    msg->header = header;

    const vector3f angular_velocity = angular_velocity_.value();
    // Asctec uses x:front y:right z:down, we use x:front, y:left, z:up.
    msg->angular_velocity.x = angular_velocity.x;
    msg->angular_velocity.y = -angular_velocity.y;
    msg->angular_velocity.z = -angular_velocity.z;

    const vector3f acceleration = acceleration_.value();
    // Asctec uses x:front y:right z:down, we use x:front, y:left, z:up.
    msg->linear_acceleration.x = acceleration.x;
    msg->linear_acceleration.y = -acceleration.y;
    msg->linear_acceleration.z = -acceleration.z;

    const quaternion q_tmp = attitude_.value();
    const Eigen::Quaterniond q_trinity(q_tmp.w, q_tmp.x, q_tmp.y, q_tmp.z);
    const Eigen::Quaterniond q = helper::asctecQuaternionToRos(q_trinity);
    msg->orientation.w = q.w();
    msg->orientation.x = q.x();
    msg->orientation.y = q.y();
    msg->orientation.z = q.z();

    helper::setDiagonal3dCovariance(0.0, &(msg->orientation_covariance)); // covariance unknown
    helper::setDiagonal3dCovariance(angular_velocity_variance_, &(msg->angular_velocity_covariance));
    helper::setDiagonal3dCovariance(linear_acceleration_variance_, &(msg->linear_acceleration_covariance));

    imu_pub_.publish(msg);
  }

  const size_t n_imu_raw_subscribers = imu_pub_raw_.getNumSubscribers();

  if (n_imu_raw_subscribers > 0 && publish_raw_imu_) {
    sensor_msgs::ImuPtr msg(new sensor_msgs::Imu);
    msg->header = header;

    const vector3i angular_velocity = angular_velocity_raw_.value();
    constexpr double angvel_max_sensor_range = 2000.0;
    constexpr double guessed_scaling_factor = 50.0;
    constexpr double raw_angvel_to_si = 1.0 / angvel_max_sensor_range * M_PI / 180.0 * guessed_scaling_factor;

    // Asctec uses x:front y:right z:down, we use x:front, y:left, z:up.
    msg->angular_velocity.x = static_cast<double>(angular_velocity.x) * raw_angvel_to_si;
    msg->angular_velocity.y = -static_cast<double>(angular_velocity.y) * raw_angvel_to_si;
    msg->angular_velocity.z = -static_cast<double>(angular_velocity.z) * raw_angvel_to_si;

    const vector3i acceleration = acceleration_raw_.value();
    constexpr double acc_max_sensor_range = 2.0 * 9.81;
    constexpr double raw_acc_to_si = 1.0 / acc_max_sensor_range / 10.0;
    // Asctec uses x:front y:right z:down, we use x:front, y:left, z:up.
    msg->linear_acceleration.x = static_cast<double>(acceleration.x) * raw_acc_to_si;
    msg->linear_acceleration.y = -static_cast<double>(acceleration.y) * raw_acc_to_si;
    msg->linear_acceleration.z = -static_cast<double>(acceleration.z) * raw_acc_to_si;

    msg->orientation_covariance[0] = -1.0;  // Attitude not available.
    helper::setDiagonal3dCovariance(0.0, &(msg->angular_velocity_covariance)); // We don't know the covariance.
    helper::setDiagonal3dCovariance(0.0, &(msg->linear_acceleration_covariance));  // We don't know the covariance.

    imu_pub_raw_.publish(msg);
  }

  const size_t n_imu_nav1_subscribers = imu_pub_nav1_.getNumSubscribers();

  if (n_imu_nav1_subscribers > 0 && publish_nav1_imu_) {
    sensor_msgs::ImuPtr msg(new sensor_msgs::Imu);
    msg->header = header;

    const vector3f angular_velocity = angular_velocity_nav1_.value();
    // Asctec uses x:front y:right z:down, we use x:front, y:left, z:up.
    msg->angular_velocity.x = angular_velocity.x;
    msg->angular_velocity.y = -angular_velocity.y;
    msg->angular_velocity.z = -angular_velocity.z;

    const vector3f acceleration = acceleration_nav1_.value();
    // Asctec uses x:front y:right z:down, we use x:front, y:left, z:up.
    msg->linear_acceleration.x = acceleration.x;
    msg->linear_acceleration.y = -acceleration.y;
    msg->linear_acceleration.z = -acceleration.z;

    msg->orientation_covariance[0] = -1.0;  // Attitude not available.
    helper::setDiagonal3dCovariance(0.0, &(msg->angular_velocity_covariance)); // We don't know the covariance.
    helper::setDiagonal3dCovariance(0.0, &(msg->linear_acceleration_covariance));  // We don't know the covariance.

    imu_pub_nav1_.publish(msg);
  }
}

void CommonDataPublisher::publishMotors(const std_msgs::Header& header) {


  if (motors_pub_.getNumSubscribers() > 0) {
    mav_msgs::ActuatorsPtr msg(new mav_msgs::Actuators);
    msg->header = header;

    msg->angular_velocities.resize(kNumberOfRotors);

    for (size_t i = 0; i < kNumberOfRotors; ++i) {
      msg->angular_velocities[i] =
          static_cast<mav_msgs::Actuators::_angular_velocities_type::value_type>(motor_speed_measured_[i].value())
              * kRPM2RadPerSec;
    }

    motors_pub_.publish(msg);
  }

  if (publish_commanded_motor_speeds_ && motors_cmd_pub_.getNumSubscribers() > 0) {
    mav_msgs::ActuatorsPtr msg(new mav_msgs::Actuators);
    msg->header = header;

    msg->angular_velocities.resize(kNumberOfRotors);

    for (size_t i = 0; i < kNumberOfRotors; ++i) {
      msg->angular_velocities[i] =
          static_cast<mav_msgs::Actuators::_angular_velocities_type::value_type>(motor_speed_commanded_[i].value())
              * kRPM2RadPerSec;
    }

    motors_cmd_pub_.publish(msg);
  }
}

void CommonDataPublisher::publishDischargeCurrent(const std_msgs::Header& header) {
  std_msgs::Float32Ptr msg(new std_msgs::Float32);
  constexpr float trinityCurrentToAmpere = 1.0e-3;
  msg->data = static_cast<float>(discharge_current_.value()) * trinityCurrentToAmpere;
  discharge_current_pub_.publish(msg);
}

inline float trinityRcToJoy(int16_t trinity_rc) {
  constexpr int16_t half_stick = 1 << 14;  // full range is positive int16, i.e. 0...32768 (2^15)
  constexpr float half_stick_float = static_cast<float>(half_stick);
  return static_cast<float>(trinity_rc - half_stick) / half_stick_float;
}

void CommonDataPublisher::publishRc(const std_msgs::Header& header) {
  if (rc_pub_.getNumSubscribers() > 0) {
    sensor_msgs::JoyPtr msg(new sensor_msgs::Joy);
    msg->header = header;

    constexpr size_t n_channels = 8;
    msg->axes.resize(n_channels);
    // Stick configuration
    // Channel | Stick        | Direction
    // 0       | Right        | up = 1, down = -1
    // 1       | Right        | left = 1, right = -1
    // 2       | Left         | up = 1, down = -1
    // 3       | Left         | left = 1, right = -1
    // 4       | G (serial on)| To operator = 1, Away from operator = -1
    // 5       | Control      | Mode  To operator = 1 (pos ctrl), Mid-way (alt ctrl) = 0,  Away from operator (manual) = -1
    // 6       | Wheel        | don't know yet
    msg->axes[0] = -trinityRcToJoy(rc_pitch_.value()); // on the rc: back +1
    msg->axes[1] = -trinityRcToJoy(rc_roll_.value()); // on the rc: right +1
    msg->axes[2] = trinityRcToJoy(rc_thrust_.value()); // on the rc: full thrust +1
    msg->axes[3] = -trinityRcToJoy(rc_yaw_.value()); // on the rc: right +1
    msg->axes[4] = trinityRcToJoy(rc_mode_.value()); // on the rc: position +1
    msg->axes[5] = trinityRcToJoy(rc_power_on_off_.value()); // on the rc: up +1
    msg->axes[6] = trinityRcToJoy(rc_external_command_switch_.value()); // on the rc: enabled +1
    msg->axes[7] = trinityRcToJoy(rc_safety_pilot_switch_.value()); // on the rc: enabled +1

    msg->buttons.resize(1);
    msg->buttons[0] = rc_data_lock_.value(); // This helps determining whether the rc is switched on.

    rc_pub_.publish(msg);
  }
}

void CommonDataPublisher::publishStatus(const std_msgs::Header& header){
  using aci::MavType;
  if (status_pub_.getNumSubscribers() > 0) {
    mav_msgs::StatusPtr msg(new mav_msgs::Status);

    msg->header = header;

    constexpr float trinityVoltageToVolt = 1.0e-3;
    msg->battery_voltage = static_cast<float>(battery_voltage_.value()) * trinityVoltageToVolt;

    constexpr float trinityFlightTimeToSeconds = 0.1;
    msg->flight_time = static_cast<float>(flight_time_.value()) * trinityFlightTimeToSeconds;
    msg->system_uptime = helper::trinity::convertTime(fcu_time_us_.value());

    constexpr float cpuTimeToSeconds = 1.0e-6;
    msg->cpu_load = static_cast<float>(cpu_time_.value()) * cpuTimeToSeconds
        * static_cast<float>(aci::kDefaultTrinityRate);

    msg->vehicle_type = aci::mavTypeToString(aci::instance().getMavType());

    msg->rc_command_mode = aci::flightModeToStringTrinity(flight_mode_.value());

    double generated_thrust = 0;
    bool motors_running = false;
    for (size_t i = 0; i < kNumberOfRotors; ++i) {
      if(motor_speed_measured_[i].value() >= kMinMotorSpeed){
        motors_running = true;
      }
      generated_thrust += kMotorConstant*(motor_speed_measured_[i].value() * kRPM2RadPerSec)*(motor_speed_measured_[i].value() * kRPM2RadPerSec);
    }

    // sets motor status (note will not output starting or stopping)
    if (motors_running){
      msg->motor_status = "running";
    }
    else{
      msg->motor_status = "stopped";
    }

    if (std::abs(generated_thrust - commanded_thrust_) < 2.0
        && motor_speed_measured_[0].value() > 0)  // if commanded thrust is tracked within 2N and motors are spinning then we assume we are in air
      msg->in_air = true;
    else
      msg->in_air = false;

    // TODO(acmarkus): serial interface enabled?

    status_pub_.publish(msg);
  }

  if (safety_pilot_status_pub_.getNumSubscribers() > 0) {
    std_msgs::StringPtr msg(new std_msgs::String);
    msg->data = aci::safetyPilotStateToString(safety_pilot_status_.value());
    safety_pilot_status_pub_.publish(msg);
  }
}

}  // end namespace trinity

}  // end namespace ethzasl_mav_interface
