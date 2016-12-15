#include "euroc_vicon_transmission/vicon_pub.h"

namespace euroc_vicon_transmission {
ViconPub::ViconPub(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      num_subscribers_odometry_(0),
      num_subscribers_transform_(0),
      port_number_(5555),
      publisher_buffer_(1000),
      vicon_odometry_topic_("/do_not_subscribe/forbidden_vicon_odometry"),
      vicon_transform_topic_("/do_not_subscribe/forbidden_vicon_transform"),
	  num_odometry_subscribers_topic_("/do_not_subscribe/num_odometry_subscribers"),
	  num_transform_subscribers_topic_("/do_not_subscribe/num_transform_subscribers")
	  {
  setParametersFromRos();
  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>(
      nh_.getNamespace() + vicon_odometry_topic_, publisher_buffer_);
  transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(
      nh_.getNamespace() + vicon_transform_topic_, publisher_buffer_);

  num_odometry_subscribers_pub_ = nh_.advertise<std_msgs::Int32>(
      nh_.getNamespace() + num_odometry_subscribers_topic_, publisher_buffer_);
  num_transform_subscribers_pub_ = nh_.advertise<std_msgs::Int32>(
      nh_.getNamespace() + num_transform_subscribers_topic_, publisher_buffer_);
}

void ViconPub::setParametersFromRos() {
  nh_private_.param("vicon_odometry_topic", vicon_odometry_topic_, vicon_odometry_topic_);
  nh_private_.param("vicon_transform_topic", vicon_transform_topic_, vicon_transform_topic_);
  nh_private_.param("num_odometry_subscribers_topic", num_odometry_subscribers_topic_, num_odometry_subscribers_topic_);
  nh_private_.param("num_transform_subscribers_topic", num_transform_subscribers_topic_, num_transform_subscribers_topic_);
  nh_private_.param("publisher_buffer", publisher_buffer_, publisher_buffer_);
  nh_private_.param("port_number", port_number_, port_number_);
}

void ViconPub::deserializeOdometryMsg(const std::string& serial_data,
                                      nav_msgs::Odometry* msg) {
  Odometry odometry;
  odometry.ParseFromString(serial_data);

  // Set header.
  msg->header.seq = odometry.header().seq();
  msg->header.stamp.sec = odometry.header().stamp().sec();
  msg->header.stamp.nsec = odometry.header().stamp().nsec();
  msg->header.frame_id = odometry.header().frame_id();

  // Set child_frame_id.
  msg->child_frame_id = odometry.child_frame_id();

  // Set pose.
  msg->pose.pose.position.x = odometry.pose().pose().position().x();
  msg->pose.pose.position.y = odometry.pose().pose().position().y();
  msg->pose.pose.position.z = odometry.pose().pose().position().z();

  msg->pose.pose.orientation.x = odometry.pose().pose().orientation().x();
  msg->pose.pose.orientation.y = odometry.pose().pose().orientation().y();
  msg->pose.pose.orientation.z = odometry.pose().pose().orientation().z();
  msg->pose.pose.orientation.w = odometry.pose().pose().orientation().w();

  for (size_t i = 0; i < odometry.pose().covariance_size(); i++) {
    msg->pose.covariance[i] = odometry.pose().covariance(i);
  }

  // Set twist.
  msg->twist.twist.linear.x = odometry.twist().twist().linear().x();
  msg->twist.twist.linear.y = odometry.twist().twist().linear().y();
  msg->twist.twist.linear.z = odometry.twist().twist().linear().z();

  msg->twist.twist.angular.x = odometry.twist().twist().angular().x();
  msg->twist.twist.angular.y = odometry.twist().twist().angular().y();
  msg->twist.twist.angular.z = odometry.twist().twist().angular().z();

  for (size_t i = 0; i < odometry.twist().covariance_size(); i++) {
    msg->twist.covariance[i] = odometry.twist().covariance(i);
  }
}

geometry_msgs::TransformStamped ViconPub::convertOdometryMsgToTranformMsg(const nav_msgs::Odometry& odo_msg) {
	geometry_msgs::TransformStamped transform_msg;
	transform_msg.header.stamp = odo_msg.header.stamp;
	transform_msg.header.frame_id = odo_msg.header.frame_id;
	transform_msg.transform.translation.x = odo_msg.pose.pose.position.x;
	transform_msg.transform.translation.y = odo_msg.pose.pose.position.y;
	transform_msg.transform.translation.z = odo_msg.pose.pose.position.z;

	transform_msg.transform.rotation.w = odo_msg.pose.pose.orientation.w;
	transform_msg.transform.rotation.x = odo_msg.pose.pose.orientation.x;
	transform_msg.transform.rotation.y = odo_msg.pose.pose.orientation.y;
	transform_msg.transform.rotation.z = odo_msg.pose.pose.orientation.z;

	return transform_msg;
}

bool ViconPub::notifyOnNumSubscribersChange() {
  if ((odometry_pub_.getNumSubscribers() - num_subscribers_odometry_) != 0) {
    uint32_t num_subscribers_prev = num_subscribers_odometry_;
    num_subscribers_odometry_ = odometry_pub_.getNumSubscribers();
    ROS_INFO(
        "Number of Vicon Odometry subscribers has changed from %d to %d subscriber(s).",
        num_subscribers_prev, num_subscribers_odometry_);

    std_msgs::Int32 msg;
    msg.data = num_subscribers_odometry_;
    num_odometry_subscribers_pub_.publish(msg);
    return true;
  }

  if ((transform_pub_.getNumSubscribers() - num_subscribers_transform_) != 0) {
    uint32_t num_subscribers_prev = num_subscribers_transform_;
    num_subscribers_transform_ = transform_pub_.getNumSubscribers();
    ROS_INFO(
        "Number of Vicon Transform subscribers has changed from %d to %d subscriber(s).",
        num_subscribers_prev, num_subscribers_transform_);

    std_msgs::Int32 msg;
    msg.data = num_subscribers_transform_;
    num_transform_subscribers_pub_.publish(msg);
    return true;
  }

  return false;
}

void ViconPub::publishOdometry(const nav_msgs::Odometry& msg) {
  odometry_pub_.publish(msg);
}

void ViconPub::publishTransform(const geometry_msgs::TransformStamped& msg) {
  transform_pub_.publish(msg);
}

}  // ns: euroc_vicon_transmission

int main(int argc, char** argv) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ros::init(argc, argv, "vicon_publisher");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  euroc_vicon_transmission::ViconPub vicon_pub(nh, nh_private);

  ROS_INFO("Initialized vicon publishing server.");

  // Create socket server.
  euroc_vicon_transmission::ServerSocket server(vicon_pub.getPortNumber());

  // Receive data from vicon side and publish vicon data.
  try {
    while (ros::ok()) {
      std::string serial_data;
      server >> serial_data;
      // Process data here.
      nav_msgs::Odometry msg;
      vicon_pub.deserializeOdometryMsg(serial_data, &msg);
      vicon_pub.publishOdometry(msg);
      geometry_msgs::TransformStamped transform_msg;
      transform_msg = vicon_pub.convertOdometryMsgToTranformMsg(msg);
      vicon_pub.publishTransform(transform_msg);

      vicon_pub.notifyOnNumSubscribersChange();
    }
  } catch (euroc_vicon_transmission::SocketException& e) {
    ROS_WARN("Exception was caught: %s\nExiting.", e.description().c_str());
  }

  ros::spin();
}
