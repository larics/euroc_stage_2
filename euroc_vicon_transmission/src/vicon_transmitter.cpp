#include "euroc_vicon_transmission/vicon_transmitter.h"

namespace euroc_vicon_transmission {
ViconTransmitter::ViconTransmitter(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      vicon_vrpn_topic_("/do_not_subscribe/forbidden_vicon_odometry"),
      broadcast_ip_("127.255.255.255"),
      callback_buffer_(1000),
      port_number_(5555) {
  setParametersFromRos();
  vicon_subscriber_ = nh_private_.subscribe(
      nh_.getNamespace() + vicon_vrpn_topic_, callback_buffer_,
      &ViconTransmitter::receiveViconCallback, this);
}

void ViconTransmitter::setParametersFromRos() {
  nh_private_.param("vicon_vrpn_topic", vicon_vrpn_topic_, vicon_vrpn_topic_);
  nh_private_.param("broadcast_ip", broadcast_ip_, broadcast_ip_);
  nh_private_.param("callback_buffer", callback_buffer_, callback_buffer_);
  nh_private_.param("port_number", port_number_, port_number_);
}

void ViconTransmitter::receiveViconCallback(
    const nav_msgs::Odometry::ConstPtr& vicon_msg) {
  try {
    std::string serial_data;
    serializeOdometryMsg(vicon_msg, &serial_data);
    ClientSocket client(broadcast_ip_, port_number_);
    client << serial_data;
  } catch (SocketException& e) {
    ROS_WARN("Exception was caught: %s", e.description().c_str());
  }
}

void ViconTransmitter::serializeOdometryMsg(
    const nav_msgs::Odometry::ConstPtr& msg, std::string* serial_data) {
  Odometry odometry;

  // Set header.
  odometry.mutable_header()->set_seq(msg->header.seq);
  odometry.mutable_header()->mutable_stamp()->set_sec(msg->header.stamp.sec);
  odometry.mutable_header()->mutable_stamp()->set_nsec(msg->header.stamp.nsec);
  odometry.mutable_header()->set_frame_id(msg->header.frame_id);

  // Set child_frame_id.
  odometry.set_child_frame_id(msg->child_frame_id);

  // Set pose.
  odometry.mutable_pose()->mutable_pose()->mutable_position()->set_x(
      msg->pose.pose.position.x);
  odometry.mutable_pose()->mutable_pose()->mutable_position()->set_y(
      msg->pose.pose.position.y);
  odometry.mutable_pose()->mutable_pose()->mutable_position()->set_z(
      msg->pose.pose.position.z);

  odometry.mutable_pose()->mutable_pose()->mutable_orientation()->set_x(
      msg->pose.pose.orientation.x);
  odometry.mutable_pose()->mutable_pose()->mutable_orientation()->set_y(
      msg->pose.pose.orientation.y);
  odometry.mutable_pose()->mutable_pose()->mutable_orientation()->set_z(
      msg->pose.pose.orientation.z);
  odometry.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(
      msg->pose.pose.orientation.w);

  for (size_t i = 0; i < msg->pose.covariance.size(); i++) {
    odometry.mutable_pose()->add_covariance(msg->pose.covariance[i]);
  }

  // Set twist.
  odometry.mutable_twist()->mutable_twist()->mutable_linear()->set_x(
      msg->twist.twist.linear.x);
  odometry.mutable_twist()->mutable_twist()->mutable_linear()->set_y(
      msg->twist.twist.linear.y);
  odometry.mutable_twist()->mutable_twist()->mutable_linear()->set_z(
      msg->twist.twist.linear.z);

  odometry.mutable_twist()->mutable_twist()->mutable_angular()->set_x(
      msg->twist.twist.angular.x);
  odometry.mutable_twist()->mutable_twist()->mutable_angular()->set_y(
      msg->twist.twist.angular.y);
  odometry.mutable_twist()->mutable_twist()->mutable_angular()->set_z(
      msg->twist.twist.angular.z);

  for (size_t i = 0; i < msg->twist.covariance.size(); i++) {
    odometry.mutable_twist()->add_covariance(msg->twist.covariance[i]);
  }

  if (!odometry.IsInitialized()) {
    ROS_WARN("At least one odometry field has not been set.");
  }
  // Serialize.
  odometry.SerializeToString(serial_data);
}
}  // ns: euroc_vicon_transmission

int main(int argc, char** argv) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ros::init(argc, argv, "vicon_transmitter");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  euroc_vicon_transmission::ViconTransmitter vicon_transmitter(nh, nh_private);

  ROS_INFO("Initialized vicon transmitter client.");

  ros::spin();
}
