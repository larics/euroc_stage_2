
#include <euroc_stage2/task3_server.h>

namespace euroc_stage2 {

Task3Server::Task3Server(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : mode_(IDLE),
      waypoint_number_(0),
      saver_constraints_violated_flag_(false),
      half_score_(false) {
  bool trajectory_from_waypoint = true;

  private_nh.param("trajectory_file_path", trajectory_file_path_,
                   kDefaultTrajectoryFilePath);
  private_nh.param("waypoint_file_path", waypoint_file_path_,
                   kDefaultWaypointFilePath);
  private_nh.param("half_score_service_name", half_score_service_name_,
                   kDefaultHalfScoreServiceName);

  start_publishing_service_ = private_nh.advertiseService(
      "start_publishing", &Task3Server::startPublishing, this);

  pos_hold_client_ = nh.serviceClient<std_srvs::Empty>("back_to_position_hold");

  transform_sub_ =
      nh.subscribe("vrpn_client/pose", 1, &Task3Server::poseCallback, this);
  saver_constraints_violated_flag_sub_ =
      nh.subscribe("saver_constraints_violated", 1,
                   &Task3Server::saverConstraintsViolatedFlagCallback, this);
  pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
      "pose", kPublisherQueueSize, true);
  marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
      "path", kPublisherQueueSize, true);
  path_pub_ = nh.advertise<planning_msgs::PolynomialTrajectory4D>(
      "path_segments", kPublisherQueueSize);
  half_score_vicon_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
      "half_score_vicon_pose", kPublisherQueueSize, true);

  vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("waypoint_marker",
                                                           kPublisherQueueSize);

  command_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, kPublisherQueueSize);

  waypoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
      mav_msgs::default_topics::COMMAND_POSE, kPublisherQueueSize);

  // read exploration trajectory (first part)
  getAbsoluteFilePath(kPackageName, &trajectory_file_path_);

  if (trajectory_from_waypoint) {
    TrajectoryInterface::readWaypointsFromFile(trajectory_file_path_,
                                               &trajectory_waypoints_,
                                               &trajectory_segment_times_);

    TrajectoryInterface::getTrajectoryFromWaypoints(
        trajectory_waypoints_, trajectory_segment_times_, &trajectory_position_,
        &trajectory_yaw_);

  } else {
    mav_planning_utils::polynomialTrajectoryFromFile<
        kDefaultPolynomialCoefficients>(
        trajectory_file_path_, &trajectory_position_, &trajectory_yaw_);
  }

  plotTrajectory();

  // read waypoints for second part
  getAbsoluteFilePath(kPackageName, &waypoint_file_path_);

  TrajectoryInterface::readWaypointsFromFile(waypoint_file_path_, &waypoints_,
                                             &waypoint_times_);

  idle_mode_ = true;
}

Task3Server::~Task3Server() {}

void Task3Server::saverConstraintsViolatedFlagCallback(
    const std_msgs::BoolConstPtr& msg) {
  saver_constraints_violated_flag_ = msg->data;
}

void Task3Server::poseCallback(const geometry_msgs::TransformStamped& msg) {
  ROS_INFO_ONCE("[task3]: Got first pose from vicon.");

  mav_msgs::eigenTrajectoryPointFromTransformMsg(msg, &current_pose_);

  //give vicon while halving score
  if(half_score_vicon_pub_.getNumSubscribers() > 0){
    if(!half_score_){

      //tell eval node to halve score
      std_srvs::Empty::Request request;
      std_srvs::Empty::Response response;
      if(!ros::service::call(half_score_service_name_, request, response)){
        ROS_ERROR("Failed to halve score");
      }
      else {
        half_score_ = true;
      }
    }

    // Publish message.
    geometry_msgs::PoseStamped msg_pub;
    mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(current_pose_,
                                                      &msg_pub);
    half_score_vicon_pub_.publish(msg_pub);

  }


  switch (mode_) {
    case Task3Mode::PUBLISH_POSE: {
      if (vicon_msg_counter_ < kNumViconMsgs) {
        // Publish message.
        geometry_msgs::PoseStamped msg_pub;
        mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(current_pose_,
                                                         &msg_pub);
        pose_pub_.publish(msg_pub);
        ++vicon_msg_counter_;
      } else {
        // Switch to trajectory mode once we are done publishing poses.
        mode_ = Task3Mode::PUBLISH_TRAJECTORY;
        vicon_msg_counter_ = 0;
      }
      break;
    }
    case Task3Mode::PUBLISH_TRAJECTORY: {
      ROS_INFO("[task3]: Publishing exploration trajectory.");

      planning_msgs::PolynomialTrajectory4D polynomial_msg;
      mav_planning_utils::trajectoryToPolynomialTrajectoryMsg(
          trajectory_position_, trajectory_yaw_, &polynomial_msg);
      path_pub_.publish(polynomial_msg);

      // publish trajecory.
      mav_msgs::EigenTrajectoryPoint::Vector flat_states;
      mav_planning_utils::sampleWholeTrajectory(
          *trajectory_position_, *trajectory_yaw_, kSamplingDt, &flat_states);

      trajectory_msgs::MultiDOFJointTrajectory msg;
      msgMultiDofJointTrajectoryFromEigen(flat_states, &msg);

      trajectory_start_time_ = ros::Time::now();

      command_pub_.publish(msg);
      mode_ = Task3Mode::WAIT_FOR_TRAJECTORY_FINISHED;
      break;
    }
    case Task3Mode::WAIT_FOR_TRAJECTORY_FINISHED: {
      ros::Duration time_diff = ros::Time::now() - trajectory_start_time_;
      if (time_diff.toSec() >
          trajectory_position_->getMaxTime() + kWaitTimeTrajectory) {
        mode_ = Task3Mode::PUBLISH_SECOND_POSE;
      }
      break;
    }
    case Task3Mode::PUBLISH_SECOND_POSE: {
      if (vicon_msg_counter_ < kNumViconMsgs) {
        // Publish message.
        geometry_msgs::PoseStamped msg_pub;
        mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(current_pose_,
                                                         &msg_pub);
        pose_pub_.publish(msg_pub);
        ++vicon_msg_counter_;
      } else {
        waypoint_number_ = 0;
        publishWaypoint();
        mode_ = Task3Mode::PUBLISH_WAYPOINT;
      }
      break;
    }
    case Task3Mode::PUBLISH_WAYPOINT: {
      if (waypoint_ == nullptr) {
        ROS_WARN_STREAM(
            "[Task3]: Waypoint does not exist, this should not happen!!!");
        break;
      }

      bool waypoint_reached =
          waypoint_->updateStatus(current_pose_.position_W, msg.header.stamp,
                                  !saver_constraints_violated_flag_);

      vis_pub_.publish(waypoint_->getMarkers());

      if (waypoint_reached) {
        waypoint_finish_times_.push_back(waypoint_->getFinishTime());
        ++waypoint_number_;
        if (waypoint_number_ < waypoints_.size()) {
          publishWaypoint();
        } else {
          mode_ = Task3Mode::IDLE;
        }
      }

      break;
    }
    case Task3Mode::IDLE: {
      break;
    }
    default: {
      ROS_ERROR(
          "[Task3]: Entered an invalid state, this should never happen and "
          "something is seriously wrong");
    }
  }
}

void Task3Server::publishWaypoint(void) {
  Eigen::Vector3d waypoint = waypoints_.at(waypoint_number_).block<3, 1>(0, 0);
  ros::Time start_time(ros::Time::now());
  waypoint_.reset(
      new Waypoint(waypoint, kWaypointRadius, kWaypointHoldTime, start_time));
  geometry_msgs::PoseStamped waypoint_msg;
  waypoint_msg.header.stamp = start_time;
  waypoint_msg.pose.position.x = waypoints_.at(waypoint_number_).x();
  waypoint_msg.pose.position.y = waypoints_.at(waypoint_number_).y();
  waypoint_msg.pose.position.z = waypoints_.at(waypoint_number_).z();
  waypoint_pub_.publish(waypoint_msg);
  ROS_INFO("[task3]: Publishing next Waypoint.");
}

void Task3Server::plotTrajectory() {
  ROS_INFO_STREAM("[task3]: Plotting Trajectory");

  trajectory_position_->print(std::cout);

  mav_viz::HexacopterMarker hex;
  mav_planning_utils::drawMavTrajectory(*trajectory_position_, *trajectory_yaw_,
                                        hex, 1.0, &path_markers_);

  marker_publisher_.reset(
      new mav_planning_utils::MarkerPublisher(marker_pub_, 5.0, path_markers_));
}

bool Task3Server::startPublishing(std_srvs::Empty::Request& request,
                                  std_srvs::Empty::Response& response) {
  ROS_INFO_STREAM("[task3]: Starts publishing.");

  mode_ = Task3Mode::PUBLISH_POSE;

  vicon_msg_counter_ = 0;
  subtask_ = 1;

  return true;
}
};
