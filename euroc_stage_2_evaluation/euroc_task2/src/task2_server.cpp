
#include <euroc_stage2/task2_server.h>

namespace euroc_stage2 {

Task2Server::Task2Server(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
  bool trajectory_from_waypoint = true;

  private_nh.param("file_path", file_path_, kDefaultTrajectoryFilePath);

  start_publishing_service_ = private_nh.advertiseService(
      "start_publishing", &Task2Server::startPublishing, this);

  pos_hold_client_ = nh.serviceClient<std_srvs::Empty>("back_to_position_hold");

  transform_sub_ = nh.subscribe("vrpn_client/pose", 1,
                                &Task2Server::poseCallback, this);
  pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/pose", 10, true);
  marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/path", 10, true);
  path_pub_ = nh.advertise<planning_msgs::PolynomialTrajectory4D>("/path_segments", 10, true);

  command_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1, true);

  std::string packagePath = ros::package::getPath("euroc_task2");
  std::stringstream ss;
  ss.str("");

  if (file_path_[0] == '/') {
    ss << file_path_;
  } else {
    ss << packagePath << '/' << file_path_;
  }

  file_path_ = ss.str();

  if (trajectory_from_waypoint) {
    TrajectoryInterface::readWaypointsFromFile(file_path_, &waypoints_, &segment_times_);

    TrajectoryInterface::getTrajectoryFromWaypoints(waypoints_, segment_times_,
                               &trajectory_position_, &trajectory_yaw_);

  } else {
    mav_planning_utils::polynomialTrajectoryFromFile<
        kDefaultPolynomialCoefficients>(file_path_, &trajectory_position_,
                                        &trajectory_yaw_);
  }

  plotTrajectory();

  idle_mode_ = true;
}

Task2Server::~Task2Server() {}



void Task2Server::poseCallback(const geometry_msgs::TransformStampedConstPtr& msg) {
  ROS_INFO_ONCE("[task2]: Got first pose from vicon.");

  mav_msgs::eigenTrajectoryPointFromTransformMsg(*msg, &current_pose_);

  if (!idle_mode_) {
    if (vicon_msg_counter_ < kNumViconMsgs) {
      // Publish message.
      geometry_msgs::PoseStamped msg;
      mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(current_pose_, &msg);
      pose_pub_.publish(msg);
      ++vicon_msg_counter_;
    } else {

      // Call the service call to takeover publishing commands.
      if (pos_hold_client_.exists()) {
        std_srvs::Empty empty_call;
        ROS_INFO("[task3]: Sending position hold service call.");
        pos_hold_client_.call(empty_call);
      }

      // TODO(burrimi): publish 5 subtask trajectories. For now only have one.
      ROS_INFO("[task2]: Publishing trajectory.");

      // publish trajecory.
      mav_msgs::EigenTrajectoryPoint::Vector flat_states;
      mav_planning_utils::sampleWholeTrajectory(
          *trajectory_position_, *trajectory_yaw_, 0.01, &flat_states);

      trajectory_msgs::MultiDOFJointTrajectory msg;
      msgMultiDofJointTrajectoryFromEigen(flat_states, &msg);
      command_pub_.publish(msg);

      // switch back to idle mode.
      idle_mode_ = true;
    }
  }
}

void Task2Server::plotTrajectory() {
  ROS_INFO_STREAM("Publishing Trajectory");

  planning_msgs::PolynomialTrajectory4D msg;
  mav_planning_utils::trajectoryToPolynomialTrajectoryMsg(
      trajectory_position_, trajectory_yaw_, &msg);

  trajectory_position_->print(std::cout);

  path_pub_.publish(msg);

  mav_viz::HexacopterMarker hex;
  mav_planning_utils::drawMavTrajectory(*trajectory_position_, *trajectory_yaw_,
                                        hex, 1.0, &path_markers_);

  marker_publisher_.reset(
      new mav_planning_utils::MarkerPublisher(marker_pub_, 5.0, path_markers_));
}



bool Task2Server::startPublishing(std_srvs::Empty::Request& request,
                                  std_srvs::Empty::Response& response) {
  ROS_INFO_STREAM("[task2]: Starts publishing.");

  idle_mode_ = false;

  vicon_msg_counter_ = 0;
  subtask_ = 1;

  return true;
}
};
