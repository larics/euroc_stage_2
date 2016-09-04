#ifndef EUROC_TASK4_TASK4_SERVER_H
#define EUROC_TASK4_TASK4_SERVER_H

#include <Eigen/Eigen>
#include <stdio.h>
#include <fstream>
#include <iostream>

// ros
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

// ros msgs
#include <mav_msgs/default_topics.h>
#include <mav_msgs/conversions.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_srvs/Empty.h>

namespace euroc_stage2 {

class Task4Server {
 public:
  static constexpr double kMaxYawTolerance = 0.35; // Radians.

  Task4Server(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~Task4Server();

 private:
  bool readWaypointsFromCsv(const std::string& filename);

  // Service and subscriber callbacks.
  bool startPublishing(std_srvs::Empty::Request& request,
                       std_srvs::Empty::Response& response);

  void odometryCallback(const nav_msgs::Odometry& msg);

  void publishMarkers(const mav_msgs::EigenOdometry& odom,
                      const Eigen::Vector3d& desired_vel_body,
                      const Eigen::Vector3d& commanded_vel_body,
                      double commanded_yaw_rate);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Evaluate current error...
  // Output all data into a CSV file.
  // Print current/final state.

  // Subscribers
  ros::Subscriber odometry_sub_;

  // Publishers
  ros::Publisher twist_pub_;
  ros::Publisher marker_pub_;

  ros::ServiceServer start_publishing_service_;

  bool idle_mode_;

  // Task 4-specific state.
  // Settings.
  double vel_max_;
  double acc_max_;
  double yaw_rate_max_;
  double dt_sec_;
  double p_gain_;

  // Current state.
  ros::Time last_msg_time_;

  Eigen::Vector3d current_waypoint_;

  // State in the waypoint list.
  double waypoint_elapsed_time_;
  size_t waypoint_index_;

  // Some kind of list of waypoints...
  std::vector<Eigen::Vector3d> waypoint_positions_;
  std::vector<double> waypoint_times_sec_;
};

}  // namespace euroc_stage2

#endif  // EUROC_TASK4_TASK4_SERVER_H

