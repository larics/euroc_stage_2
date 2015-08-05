/*
 * safety_pose_publisher.hpp
 *
 *  Created on: 18.06.2015
 *      Author: burrimi
 */

#ifndef INCLUDE_SAFETY_POSE_PUBLISHER_HPP_
#define INCLUDE_SAFETY_POSE_PUBLISHER_HPP_


#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <aci/variable.h>
#include <aci/uart.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <ethzasl_mav_interface/helper.h>

namespace mav_saver {


// Default values
constexpr int kDefaultMeasurementDivisor = 1;
static const std::string kDefaultSerialPort = "/dev/ttyUSB0";
constexpr int kDefaultBaudrate = 57600;


class SafetyPosePublisher {
 public:
  SafetyPosePublisher(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  SafetyPosePublisher(int baudrate, std::string port, int measurement_divisor);
  ~SafetyPosePublisher() {};

  bool SetupSerialPort();

  void SetPose(const Eigen::Vector3d& p_W_I, const Eigen::Quaterniond& q_W_I);
  void SetTakeControlFlag(bool take_control_flag);

 private:
  static unsigned short CrcUpdate(unsigned short crc, unsigned char data);
  static unsigned short UpdateCrc16(unsigned short crc, const void * data, unsigned short cnt);

  std::shared_ptr<aci::RawBuffer> buffer_;
  int measurement_divisor_;
  bool take_control_;
  bool serial_port_open_;
  ros::Timer watchdog_;
  int aci_msg_counter_;
  int pose_counter_;

  int baudrate_;
  std::string port_;
};


}  // namespace mav_saver



#endif /* INCLUDE_SAFETY_POSE_PUBLISHER_HPP_ */
