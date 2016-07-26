#ifndef TASK3_SERVER_H
#define TASK3_SERVER_H

#include <Eigen/Eigen>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>

// ros
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>

// ros msgs
#include <mav_msgs/default_topics.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>

#include <euroc_stage2/trajectory_interface.h>
#include <euroc_stage2/waypoint.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_planning_utils/polynomial_optimization_nonlinear.h>
#include <mav_planning_utils/trajectory_base.h>
#include <mav_planning_utils/trajectory_types.h>
#include <mav_planning_utils/trajectory_base.h>
#include <mav_planning_utils/ros_trajectory_interface.h>
#include <mav_viz/hexacopter_marker.h>
#include <yaml-cpp/yaml.h>

namespace euroc_stage2 {

const std::string kDefaultTrajectoryFilePath = "res/trajectory.txt";
const std::string kDefaultWaypointFilePath = "res/waypoints.txt";
const std::string kPackageName = "euroc_task3";

enum Task3Mode {
  IDLE = 0,
  PUBLISH_POSE = 1,
  PUBLISH_TRAJECTORY = 2,
  WAIT_FOR_TRAJECTORY_FINISHED = 3,
  PUBLISH_SECOND_POSE = 4,
  PUBLISH_WAYPOINT = 5
};

// Task 3 consists of the following phases:
// 1) Publish Vicon Pose for 10 seconds (kNumViconMsgs at a rate of 100 Hz).
// 2) Send a collision free trajectory to the team to explore parts of the vicon
// room.
// 3) Wait for kWaitTimeTrajectory after finishing the trajectory.
// 4) Publish waypoints to which the team has to plan a path, fly there, and
// stay within a kWaypointRadius radius for kWaypointHoldTime seconds.

class Task3Server {
 public:
  static constexpr int kNumViconMsgs =
      1000;  // number of vicon messages published at the beginning of the task,
             // to calibrate the odometry of the challenger.
  static constexpr int kDefaultPolynomialCoefficients = 10;
  static constexpr int kDerivativeToOptimize =
      mav_planning_utils::derivative_order::SNAP;

  static constexpr double kSamplingDt =
      0.01;  // At what rate to sample the trajectory.
  static constexpr double kWaitTimeTrajectory = 5.0;  // Wait time after flying
                                                      // trajectory before start
                                                      // of sending waypoints.

  static constexpr unsigned int kPublisherQueueSize = 10;

  static constexpr double kWaypointRadius =
      0.35;  // Radius in which the MAV has to stay.
  static constexpr double kWaypointHoldTime =
      10.0;  // Time in seconds to hold the waypoint.

  typedef mav_planning_utils::Segment<kDefaultPolynomialCoefficients>::Vector
      Segments;
  typedef std::shared_ptr<Segments> SegmentsPtr;

  Task3Server(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  ~Task3Server();

 private:
  bool startPublishing(std_srvs::Empty::Request& request,
                       std_srvs::Empty::Response& response);

  void saverConstraintsViolatedFlagCallback(const std_msgs::BoolConstPtr& msg);

  void poseCallback(const geometry_msgs::TransformStamped& msg);

  void publishWaypoint(void);

  void plotTrajectory();

  // subscribers
  ros::Subscriber transform_sub_;
  ros::Subscriber saver_constraints_violated_flag_sub_;

  // publishers
  ros::Publisher pose_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher path_pub_;
  ros::Publisher command_pub_;
  ros::Publisher waypoint_pub_;
  ros::Publisher vis_pub_;

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
  // subtusk number 1..5
  int subtask_;

  bool idle_mode_;

  Task3Mode mode_;

  std::vector<Eigen::Vector4d> trajectory_waypoints_;
  std::vector<double> trajectory_segment_times_;

  std::vector<Eigen::Vector4d> waypoints_;
  std::vector<double> waypoint_times_;

  std::string trajectory_file_path_;
  std::string waypoint_file_path_;

  ros::Time trajectory_start_time_;

  Waypoint::Ptr waypoint_;

  unsigned int waypoint_number_;

  std::vector<ros::Duration> waypoint_finish_times_;

  bool saver_constraints_violated_flag_;
};
}
#endif
