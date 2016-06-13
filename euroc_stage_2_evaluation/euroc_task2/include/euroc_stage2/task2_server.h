#ifndef TASK2_SERVER_H
#define TASK2_SERVER_H

#include <Eigen/Eigen>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>

// ros
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "ros/package.h"

// ros msgs
#include <mav_msgs/default_topics.h>
#include <std_srvs/Empty.h>

#include <euroc_stage2/trajectory_interface.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_planning_utils/polynomial_optimization_nonlinear.h>
#include <mav_planning_utils/trajectory_base.h>
#include <mav_planning_utils/trajectory_types.h>
#include <mav_planning_utils/ros_trajectory_interface.h>
#include <mav_viz/hexacopter_marker.h>
#include <yaml-cpp/yaml.h>

namespace euroc_stage2 {

const std::string kDefaultTrajectoryFilePath = "res/task2.txt";
const int num_coeff = 10;

class Task2Server {
 public:
  static constexpr int kNumViconMsgs = 1000;
  static constexpr int kDefaultPolynomialCoefficients = 10;
  static constexpr int kDerivativeToOptimize =
      mav_planning_utils::derivative_order::SNAP;

  typedef mav_planning_utils::Segment<kDefaultPolynomialCoefficients>::Vector
      Segments;
  typedef std::shared_ptr<Segments> SegmentsPtr;

  Task2Server(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  ~Task2Server();

 private:

  bool startPublishing(std_srvs::Empty::Request& request,
                       std_srvs::Empty::Response& response);

  void poseCallback(const geometry_msgs::TransformStampedConstPtr& msg);

  void plotTrajectory();


  // subscribers
  ros::Subscriber transform_sub_;

  // publishers
  ros::Publisher pose_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher path_pub_;
  ros::Publisher command_pub_;

  // clients
  ros::ServiceClient pos_hold_client_;
  ros::ServiceServer start_publishing_service_;

  mav_planning_utils::TrajectoryBase::Ptr trajectory_position_;
  mav_planning_utils::TrajectoryBase::Ptr trajectory_yaw_;

  visualization_msgs::MarkerArray path_markers_;
  std::shared_ptr<mav_planning_utils::MarkerPublisher> marker_publisher_;

  mav_msgs::EigenTrajectoryPoint current_pose_;

  // number of published vicon messages
  int vicon_msg_counter_;
  // subtask number 1..5
  int subtask_;

  bool idle_mode_;

  std::vector<Eigen::Vector4d> waypoints_;
  std::vector<double> segment_times_;

  std::string file_path_;
};
}
#endif
