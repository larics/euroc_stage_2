// The socket client which subscribes to Vicon odometry messages on the VICON
// network and sends them to the socket server on the EUROC network.

#ifndef INCLUDE_SOCKET_TEST_VICON_TRANSMITTER_H_
#define INCLUDE_SOCKET_TEST_VICON_TRANSMITTER_H_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "./odometry_msg.pb.h"
#include "euroc_vicon_transmission/client_socket.h"
#include "euroc_vicon_transmission/socket_exception.h"

namespace euroc_vicon_transmission {
class ViconTransmitter {
 public:
  ViconTransmitter(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);
  void receiveViconCallback(const nav_msgs::Odometry::ConstPtr& vicon_msg);
  void serializeOdometryMsg(const nav_msgs::Odometry::ConstPtr& msg,
                            std::string* serial_data);

 private:
  void setParametersFromRos();

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber vicon_subscriber_;

  // Parameter
  std::string vicon_vrpn_topic_;
  std::string broadcast_ip_;
  int callback_buffer_;
  int port_number_;
};
}  // ns: euroc_vicon_transmission

#endif
