// The socket server which publishes Vicon odometry on the EUROC side.

#ifndef INCLUDE_SOCKET_TEST_VICON_PUB_H_
#define INCLUDE_SOCKET_TEST_VICON_PUB_H_

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>

#include "./odometry_msg.pb.h"
#include "euroc_vicon_transmission/server_socket.h"
#include "euroc_vicon_transmission/socket_exception.h"

namespace euroc_vicon_transmission {
const int kPortNumber = 5555;
const int kPubBuffer = 1000;
const std::string kTopic = "/neo/vrpn_client/transmitted_odometry";

class ViconPub {
 public:
  ViconPub(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  void deserializeOdometryMsg(const std::string& serial_data,
                              nav_msgs::Odometry* msg);
  void publishOdometry(const nav_msgs::Odometry& msg);
  void publishTransform(const geometry_msgs::TransformStamped& msg);
  int getPortNumber() const { return port_number_; }
  geometry_msgs::TransformStamped convertOdometryMsgToTranformMsg(const nav_msgs::Odometry& odo_msg);
  bool notifyOnNumSubscribersChange();

 private:
  void setParametersFromRos();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher odometry_pub_;
  ros::Publisher transform_pub_;

  ros::Publisher num_odometry_subscribers_pub_;
  ros::Publisher num_transform_subscribers_pub_;

  uint32_t num_subscribers_transform_;
  uint32_t num_subscribers_odometry_;
  int port_number_;
  int publisher_buffer_;
  std::string vicon_odometry_topic_;
  std::string vicon_transform_topic_;

  std::string num_odometry_subscribers_topic_;
  std::string num_transform_subscribers_topic_;
};
}  // ns: euroc_vicon_transmission

#endif
