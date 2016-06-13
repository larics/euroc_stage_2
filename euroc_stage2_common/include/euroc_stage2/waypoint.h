#ifndef EUROC_STAGE2_COMMON_WAYPOINT_H
#define EUROC_STAGE2_COMMON_WAYPOINT_H

#include <boost/filesystem.hpp>
#include <boost/math/tools/minima.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <Eigen/Eigen>

#include <fstream>
#include <memory>

namespace euroc_stage2 {

bool getAbsoluteFilePath(const std::string& package_name, std::string* file_name);

class Waypoint {
  Eigen::Vector3d position_;
  double radius_;
  ros::Duration required_wait_duration_;
  ros::Time time_when_entered_;
  ros::Time start_time_;

  ros::Duration finish_duration_;

  bool first_update_;

  visualization_msgs::MarkerArray markerArray_;

public:
  typedef std::shared_ptr<Waypoint> Ptr;

  Waypoint(const Eigen::Vector3d& postion, double radius,
           double required_wait_time, const ros::Time& start_time);

  bool updateStatus(const Eigen::Vector3d& current_position, const ros::Time& current_time, bool valid);

  bool updateStatus(const geometry_msgs::PoseStampedConstPtr& msg, bool valid);

  bool updateStatus(const geometry_msgs::TransformStampedConstPtr& msg, bool valid);

  void clearFinishTime(void);

  ros::Duration getFinishTime(void);

  void changeStartTime(const ros::Time& start_time);

  const visualization_msgs::MarkerArray& getMarkers(void);
};
}

#endif
