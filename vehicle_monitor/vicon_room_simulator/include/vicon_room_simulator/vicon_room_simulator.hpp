#ifndef INCLUDE_VICON_ROOM_SIMULATOR_HPP_
#define INCLUDE_VICON_ROOM_SIMULATOR_HPP_

#include <geometry_msgs/TransformStamped.h>
#include "gazebo_msgs/ModelStates.h"
#include <ros/ros.h>

namespace vicon_room_simulator {

// Small class that will grab every object in a gazebo simulation and publish
// its pose mimicing what would be seen by objects tracked by a vicon system.
// pose topic name = ground_truth/(gazebo object name)
class ViconRoom {
 public:
  ViconRoom(const ros::NodeHandle& nh);
  void gazeboCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);

 private:
  ros::Subscriber gazebo_sub_;
  std::map<std::string, ros::Publisher> vicon_pubs_;

  ros::NodeHandle nh_;

  ros::Time last_update_time_;
};
}

constexpr double kViconFreq = 100;
constexpr size_t kQueueSize = 100;

#endif /* INCLUDE_VICON_ROOM_SIMULATOR_HPP_ */
