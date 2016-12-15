#include <functional>
#include <random>
#include <sstream>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace euroc_vicon_transmission {
const int kPubBuffer = 1000;
const double kDoubleRandMax = 10000.0;
const std::string kTopic = "/euroc6/vrpn_client/estimated_odometry";

nav_msgs::Odometry createRandomOdometry(const int count) {
  nav_msgs::Odometry msg;
  msg.header.seq = count;
  msg.header.frame_id = "/world";
  msg.child_frame_id = "/none";

  std::mt19937::result_type seed = time(0);
  std::mt19937 uint32_rand(seed);
  auto double_rand = std::bind(
      std::uniform_real_distribution<double>(-kDoubleRandMax, kDoubleRandMax),
      std::mt19937(seed));

  msg.header.stamp = ros::Time::now();

  msg.pose.pose.position.x = double_rand();
  msg.pose.pose.position.y = double_rand();
  msg.pose.pose.position.z = double_rand();

  msg.pose.pose.orientation.x = double_rand();
  msg.pose.pose.orientation.y = double_rand();
  msg.pose.pose.orientation.z = double_rand();
  msg.pose.pose.orientation.w = double_rand();

  for (size_t i = 0; i < msg.pose.covariance.size(); i++) {
    msg.pose.covariance[i] = std::abs(double_rand());
  }

  msg.twist.twist.linear.x = double_rand();
  msg.twist.twist.linear.y = double_rand();
  msg.twist.twist.linear.z = double_rand();

  msg.twist.twist.angular.x = double_rand();
  msg.twist.twist.angular.y = double_rand();
  msg.twist.twist.angular.z = double_rand();

  for (size_t i = 0; i < msg.twist.covariance.size(); i++) {
    msg.twist.covariance[i] = std::abs(double_rand());
  }

  return msg;
}
}  // ns: euroc_vicon_transmission

int main(int argc, char **argv) {
  ros::init(argc, argv, "fake_vicon");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<nav_msgs::Odometry>(
      euroc_vicon_transmission::kTopic, euroc_vicon_transmission::kPubBuffer);

  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok()) {
    nav_msgs::Odometry msg =
        euroc_vicon_transmission::createRandomOdometry(count);
    msg.header.seq = count;

    ROS_INFO("%d", msg.header.seq);

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
