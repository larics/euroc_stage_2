#ifndef SAFE_JOYSTICK_NAV_SAFE_JOYSTICK_NAV_H
#define SAFE_JOYSTICK_NAV_SAFE_JOYSTICK_NAV_H

#include <Eigen/Eigen>
#include <stdio.h>
#include <fstream>
#include <iostream>

// ros
#include <ros/ros.h>
#include <ros/callback_queue.h>

// ros msgs
#include <mav_msgs/default_topics.h>
#include <mav_msgs/conversions.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Joy.h>

#include <octomap_world/octomap_manager.h>
namespace safe_joystick_nav {

class SafeJoystickNav {
 public:
  SafeJoystickNav(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  ~SafeJoystickNav() {}

 private:
  void odometryCallback(const nav_msgs::Odometry& msg);
  void twistCallback(const geometry_msgs::TwistStamped& msg);
  void octomapCallback(const octomap_msgs::Octomap& msg);

  // TODO(helenol): use joystick mapping from flight manager.
  void joyCallback(const sensor_msgs::Joy& msg);

  double deadZone(double axis_pos) const;

  bool enforceCollisionConstraints(
      const mav_msgs::EigenTrajectoryPoint& current_point,
      const mav_msgs::EigenTrajectoryPoint& target_point,
      mav_msgs::EigenTrajectoryPoint* safe_point);

  bool getClosestFreePosition(const Eigen::Vector3d& current_position,
                              const Eigen::Vector3d& target_position,
                              Eigen::Vector3d* safe_position);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers
  ros::Subscriber odometry_sub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber octomap_sub_;

  // Publishers
  ros::Publisher command_pub_;

  ros::ServiceServer start_publishing_service_;

  // Settings
  double vel_max_;
  double acc_max_;
  double yaw_rate_max_;
  double dt_sec_;               // dt of odometry messages/position commands.
  double command_timeout_sec_;  // How long to wait after last RC command to
  // still send control commands.
  double min_distance_;  // Minimum distance to obstacles to maintain.
  double max_carrot_distance_;
  Eigen::Vector3d robot_size_;

  // Objects
  std::shared_ptr<volumetric_mapping::OctomapManager> world_;

  // Current state
  Eigen::Vector3d desired_vel_;
  double desired_yaw_rate_;
  ros::Time last_command_time_;
};

}  // namespace safe_joystick_nav

#endif  // SAFE_JOYSTICK_NAV_SAFE_JOYSTICK_NAV_H
