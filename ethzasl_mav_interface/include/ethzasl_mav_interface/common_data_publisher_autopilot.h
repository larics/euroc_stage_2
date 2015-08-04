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

#ifndef COMMON_DATA_PUBLISHER_H_
#define COMMON_DATA_PUBLISHER_H_

#include <ros/ros.h>
#include <aci/aci.h>

namespace ethzasl_mav_interface {

namespace autopilot {

class CommonDataPublisher {
 public:
  CommonDataPublisher(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

 private:
  static constexpr int kNumberOfRotors = 6;

  void fastPacketCallback();
  void mediumPacketCallback();
  void slowPacketCallback();

  void publishImu(const std_msgs::Header& header);
  void publishStatus(const std_msgs::Header& header);
  void publishMotors(const std_msgs::Header& header);
  void publishRc(const std_msgs::Header& header);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher gps_pub_;
  ros::Publisher imu_pub_; ///< publisher for sensor_msgs/Imu message
  ros::Publisher motors_pub_; ///< publisher for motor message
  ros::Publisher rc_pub_;
  ros::Publisher status_pub_;
  ros::Publisher mag_pub_;

  aci::Variable<aci::VariableInt64> fcu_time_;

  aci::Variable<aci::VariableInt32> angle_roll_;
  aci::Variable<aci::VariableInt32> angle_pitch_;
  aci::Variable<aci::VariableInt32> angle_yaw_;

  aci::Variable<aci::VariableInt32> angvel_roll_;
  aci::Variable<aci::VariableInt32> angvel_pitch_;
  aci::Variable<aci::VariableInt32> angvel_yaw_;

  aci::Variable<aci::VariableInt16> acc_x_;
  aci::Variable<aci::VariableInt16> acc_y_;
  aci::Variable<aci::VariableInt16> acc_z_;

  aci::Variable<aci::VariableUint8> motor_speed_[kNumberOfRotors];

  aci::Subscription fast_packet_sub_;

  double angular_velocity_variance_;
  double linear_acceleration_variance_;
  std::string frame_id_;
};

}  // end namespace autopilot

}  // end namespace ethzasl_mav_interface

#endif /* COMMON_DATA_PUBLISHER_H_ */
