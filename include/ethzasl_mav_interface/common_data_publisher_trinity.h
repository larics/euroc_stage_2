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

#ifndef COMMON_DATA_PUBLISHER_TRINITY_H_
#define COMMON_DATA_PUBLISHER_TRINITY_H_

#include <ros/ros.h>
#include <aci/aci.h>

namespace ethzasl_mav_interface {

namespace trinity {

class CommonDataPublisher {
 public:
  CommonDataPublisher(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

 private:
  static constexpr int kNumberOfRotors = 6;

  void fastPacketCallback();
  void mediumPacketCallback();
  void slowPacketCallback();

  bool createHeaderFromFcuTime(int64_t fcu_time_us, std_msgs::Header* header);

  void publishImu(const std_msgs::Header& header);
  void publishStatus(const std_msgs::Header& header);
  void publishMotors(const std_msgs::Header& header);
  void publishDischargeCurrent(const std_msgs::Header& header);
  void publishRc(const std_msgs::Header& header);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher gps_pub_;
  ros::Publisher imu_pub_; ///< publisher for sensor_msgs/Imu message
  ros::Publisher motors_pub_; ///< publisher for motor message
  ros::Publisher motors_cmd_pub_; ///< publisher for commanded motor message
  ros::Publisher rc_pub_;
  ros::Publisher status_pub_;
  ros::Publisher mag_pub_;
  ros::Publisher discharge_current_pub_;
  ros::Publisher safety_pilot_status_pub_;

  aci::Variable<aci::VariableInt64> fcu_time_us_;

  aci::Variable<aci::VariableQuaternion> attitude_;
  aci::Variable<aci::VariableVector3f> angular_velocity_;
  aci::Variable<aci::VariableVector3f> acceleration_;

  aci::Variable<aci::VariableFloat> air_pressure_;
  aci::Variable<aci::VariableVector3f> magnetic_field_;
  aci::Variable<aci::VariableInt32> angvel_yaw_;

  aci::Variable<aci::VariableUint16> motor_speed_measured_[kNumberOfRotors];
  aci::Variable<aci::VariableUint16> motor_speed_commanded_[kNumberOfRotors];
  bool publish_commanded_motor_speeds_;

  aci::Variable<aci::VariableUint8> rc_data_lock_;
  aci::Variable<aci::VariableInt16> rc_roll_;
  aci::Variable<aci::VariableInt16> rc_pitch_;
  aci::Variable<aci::VariableInt16> rc_yaw_;
  aci::Variable<aci::VariableInt16> rc_thrust_;
  aci::Variable<aci::VariableInt16> rc_mode_;
  aci::Variable<aci::VariableInt16> rc_power_on_off_;
  aci::Variable<aci::VariableInt16> rc_safety_pilot_switch_;
  aci::Variable<aci::VariableInt16> rc_external_command_switch_;

  aci::Variable<aci::VariableUint32> flight_time_;
  aci::Variable<aci::VariableUint32> flight_mode_;
  aci::Variable<aci::VariableUint8> command_status_;
  aci::Variable<aci::VariableUint32> gps_satellites_;
  aci::Variable<aci::VariableInt32> gps_status_;
  aci::Variable<aci::VariableUint16> cpu_time_;
  aci::Variable<aci::VariableUint16> battery_voltage_;
  aci::Variable<aci::VariableUint16> discharge_current_;
  aci::Variable<aci::VariableUint8> safety_pilot_status_;

  aci::Subscription aci_fast_packet_sub_;
  aci::Subscription aci_medium_packet_sub_;
  aci::Subscription aci_slow_packet_sub_;

  double angular_velocity_variance_;
  double linear_acceleration_variance_;
  std::string frame_id_;
};

}  // end namespace trinity

}  // end namespace ethzasl_mav_interface

#endif /* COMMON_DATA_PUBLISHER_TRINITY_H_ */
