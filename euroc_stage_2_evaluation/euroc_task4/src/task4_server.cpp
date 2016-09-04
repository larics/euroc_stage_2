#include <visualization_msgs/MarkerArray.h>
#include <euroc_stage2/trajectory_interface.h>
#include <euroc_stage2/csv_parser.h>

#include <euroc_stage2/task4_server.h>

namespace euroc_stage2 {

Task4Server::Task4Server(const ros::NodeHandle& nh,
                         const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      idle_mode_(true),
      vel_max_(0.5),
      acc_max_(5.0),
      yaw_rate_max_(1.5),
      dt_sec_(0.05),
      p_gain_(1.0) {
  start_publishing_service_ = nh_private_.advertiseService(
      "start_publishing", &Task4Server::startPublishing, this);

  odometry_sub_ =
      nh_.subscribe("odometry", 1, &Task4Server::odometryCallback, this);

  twist_pub_ =
      nh_.advertise<geometry_msgs::TwistStamped>("twist_reference", 1, false);

  marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "markers", 1, false);

  std::string waypoint_file = "task4_waypoints.txt";
  nh_private_.param("waypoint_file", waypoint_file, waypoint_file);
  nh_private_.param("vel_max", vel_max_, vel_max_);

  if (!readWaypointsFromCsv(waypoint_file) || waypoint_positions_.empty() ||
      waypoint_times_sec_.empty()) {
    ROS_FATAL("Could not read waypoint file!");
    ros::shutdown();
    return;
  }

  waypoint_elapsed_time_ = 0.0;
  waypoint_index_ = 0;
  current_waypoint_ = waypoint_positions_.front();
}

Task4Server::~Task4Server() {}

bool Task4Server::readWaypointsFromCsv(const std::string& filename) {
  const size_t num_columns = 4;  // time, then 3 for position.
  std::vector<Eigen::VectorXd> parsed_vector;

  bool success = parseCsvIntoVector(filename, num_columns, &parsed_vector);

  waypoint_times_sec_.clear();
  waypoint_positions_.clear();
  if (success && !parsed_vector.empty()) {
    waypoint_times_sec_.resize(parsed_vector.size());
    waypoint_positions_.resize(parsed_vector.size());

    for (size_t i = 0; i < parsed_vector.size(); ++i) {
      waypoint_times_sec_[i] = parsed_vector[i](0);
      waypoint_positions_[i] = Eigen::Vector3d(
          parsed_vector[i](1), parsed_vector[i](2), parsed_vector[i](3));
    }
  }

  return success;
}

bool Task4Server::startPublishing(std_srvs::Empty::Request& request,
                                  std_srvs::Empty::Response& response) {
  ROS_INFO_STREAM("task4 starts publishing.");
  idle_mode_ = false;
  last_msg_time_ = ros::Time::now();

  return true;
}

void Task4Server::odometryCallback(const nav_msgs::Odometry& msg) {
  if (idle_mode_) {
    ROS_INFO_ONCE("Received first odometry message, but in idle mode.");
    return;
  }
  ROS_INFO_ONCE("Received first odometry message, starting publishing.");

  if (waypoint_index_ >= waypoint_positions_.size()) {
    ROS_INFO_ONCE("Finished all waypoints.");
    return;
  }

  double message_dt = (msg.header.stamp - last_msg_time_).toSec();
  if (message_dt < dt_sec_) {
    return;
  } else {
    last_msg_time_ = msg.header.stamp;
  }

  // Convert msg to Eigen...
  mav_msgs::EigenOdometry odom;
  mav_msgs::eigenOdometryFromMsg(msg, &odom);

  // Step 1: compare current position with desired waypoint.
  Eigen::Vector3d position_error = current_waypoint_ - odom.position_W;

  // Step 2: compute new velocity command (world frame) taking into account
  // limits.
  Eigen::Vector3d desired_vel_world = p_gain_ * (position_error);

  // Step 2b: figure out desired velocity and yaw rate (body frame)
  Eigen::Vector3d desired_vel_body =
      odom.orientation_W_B.inverse() * desired_vel_world;

  // Figure out what a reasonable value within acceleration and velocity
  // constraints is.

  // First check desired velocity magnitude...
  if (desired_vel_body.norm() > vel_max_) {
    desired_vel_body = vel_max_ * desired_vel_body / desired_vel_body.norm();
  }

  // Now check what kind of acceleration we need to achieve this.
  Eigen::Vector3d current_vel_body = odom.velocity_B;
  Eigen::Vector3d desired_acc_body =
      (desired_vel_body - current_vel_body) / dt_sec_;

  if (desired_acc_body.norm() > acc_max_) {
    desired_acc_body = acc_max_ * desired_acc_body / desired_acc_body.norm();
  }

  Eigen::Vector3d command_vel = current_vel_body + desired_acc_body * dt_sec_;

  // To calculate yaw rate, actually don't need absolute world yaw. (And this
  // avoids yaw wraparound issues).
  // Ideally, velocity should always be in the positive x direction in the
  // body frame. Therefore we just need to apply a yaw rate to compensate.
  double desired_yaw_offset = std::atan2(command_vel.y(), command_vel.x());
  //std::cout << "Desired yaw offset: " << desired_yaw_offset << " y: " << command_vel.y() << " x: " << command_vel.x() << std::endl;

  // If y is very small, we don't actually want any yaw offset -- the current
  // yaw is correct. This also happens when we're actually at the goal
  if ((std::abs(command_vel.y()) < 0.1 && command_vel.x() >= 0.0) || position_error.norm() < 0.1) {
    desired_yaw_offset = 0.0;
  }

  if (std::abs(desired_yaw_offset) > yaw_rate_max_) {
    desired_yaw_offset = std::copysign(yaw_rate_max_, desired_yaw_offset);
  }

  if (std::abs(desired_yaw_offset) > kMaxYawTolerance) {
    // This is also dangerous as the helicopter can simply go in a direction
    // where it cannot see anything in this case. Scale down commanded velocity
    // from the current velocity to permit this.
    if (current_vel_body.norm() > acc_max_ * dt_sec_) {
      command_vel =
          current_vel_body - current_vel_body.normalized() * acc_max_ * dt_sec_;
    } else {
      command_vel.setZero();
    }
  }

  // Step 3: send it.
  // For now assume holding yaw constant...
  geometry_msgs::TwistStamped twist_msg;
  twist_msg.header.stamp = ros::Time::now();
  twist_msg.header.frame_id = "base_link";  // ??
  twist_msg.twist.linear.x = command_vel.x();
  twist_msg.twist.linear.y = command_vel.y();
  twist_msg.twist.linear.z = command_vel.z();
  twist_msg.twist.angular.z = desired_yaw_offset;

  twist_pub_.publish(twist_msg);

  // Check if we need to move on on this waypoint.
  waypoint_elapsed_time_ += message_dt;
  if (waypoint_elapsed_time_ > waypoint_times_sec_[waypoint_index_]) {
    waypoint_elapsed_time_ = 0.0;
    waypoint_index_++;
    if (waypoint_index_ < waypoint_positions_.size()) {
      current_waypoint_ = waypoint_positions_[waypoint_index_];
    }
    ROS_INFO("[Task 4 Server] Advancing goal.");
  }

  // Publish markers.
  publishMarkers(odom, desired_vel_body, command_vel, desired_yaw_offset);
}

void Task4Server::publishMarkers(const mav_msgs::EigenOdometry& odom,
                                 const Eigen::Vector3d& desired_vel_body,
                                 const Eigen::Vector3d& commanded_vel_body,
                                 double commanded_yaw_rate) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "vicon";
  marker.header.stamp = ros::Time::now();
  marker.ns = "goal_point";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  // Super longform, fix.
  marker.pose.position.x = current_waypoint_.x();
  marker.pose.position.y = current_waypoint_.y();
  marker.pose.position.z = current_waypoint_.z();
  marker.pose.orientation.w = 1.0;
  marker.scale.x = marker.scale.y = marker.scale.z = 0.5;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.5;

  visualization_msgs::Marker desired_vel_marker;
  desired_vel_marker = marker;
  desired_vel_marker.ns = "commanded_vel";
  desired_vel_marker.type = visualization_msgs::Marker::ARROW;
  desired_vel_marker.pose.position.x = 0;
  desired_vel_marker.pose.position.y = 0;
  desired_vel_marker.pose.position.z = 0;
  desired_vel_marker.scale.x = 0.1;
  desired_vel_marker.scale.y = 0.2;
  desired_vel_marker.scale.z = 0.2;
  desired_vel_marker.color.a = 0.8;
  // We will set the arrow from points since this is simpler...

  geometry_msgs::Point point;
  point.x = odom.position_W.x();
  point.y = odom.position_W.y();
  point.z = odom.position_W.z();
  desired_vel_marker.points.push_back(point);

  // Convert desired vel body into world.
  Eigen::Vector3d desired_vel_world = odom.orientation_W_B * commanded_vel_body;
  point.x = odom.position_W.x() + desired_vel_world.x();
  point.y = odom.position_W.y() + desired_vel_world.y();
  point.z = odom.position_W.z() + desired_vel_world.z();
  desired_vel_marker.points.push_back(point);

  visualization_msgs::Marker commanded_yaw_marker;
  commanded_yaw_marker = marker;
  commanded_yaw_marker.ns = "commanded_yaw_rate";
  commanded_yaw_marker.type = visualization_msgs::Marker::ARROW;
  commanded_yaw_marker.pose.position.x = 0;
  commanded_yaw_marker.pose.position.y = 0;
  commanded_yaw_marker.pose.position.z = 0;
  commanded_yaw_marker.scale.x = 0.1;
  commanded_yaw_marker.scale.y = 0.2;
  commanded_yaw_marker.scale.z = 0.2;
  commanded_yaw_marker.color.r = 0.5;
  commanded_yaw_marker.color.g = 1.0;
  commanded_yaw_marker.color.b = 0.0;
  commanded_yaw_marker.color.a = 0.8;
  // We will set the arrow from points since this is simpler...

  point.x = odom.position_W.x();
  point.y = odom.position_W.y();
  point.z = odom.position_W.z();
  commanded_yaw_marker.points.push_back(point);

  // Convert desired vel body into world.
  Eigen::Vector3d desired_dir_world =
      odom.orientation_W_B * Eigen::Vector3d(0.0, commanded_yaw_rate, 0.0);
  point.x = odom.position_W.x() + desired_dir_world.x();
  point.y = odom.position_W.y() + desired_dir_world.y();
  point.z = odom.position_W.z() + desired_dir_world.z();
  commanded_yaw_marker.points.push_back(point);

  visualization_msgs::Marker current_vel_marker;
  current_vel_marker = desired_vel_marker;
  current_vel_marker.points.clear();
  current_vel_marker.ns = "current_vel";
  current_vel_marker.id = 2;
  current_vel_marker.color.r = 0.0;
  current_vel_marker.color.b = 1.0;
  current_vel_marker.color.g = 0.0;

  point.x = odom.position_W.x();
  point.y = odom.position_W.y();
  point.z = odom.position_W.z();
  current_vel_marker.points.push_back(point);

  Eigen::Vector3d current_vel_world = odom.getVelocityWorld();
  point.x = odom.position_W.x() + current_vel_world.x();
  point.y = odom.position_W.y() + current_vel_world.y();
  point.z = odom.position_W.z() + current_vel_world.z();
  current_vel_marker.points.push_back(point);

  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.push_back(marker);
  marker_array.markers.push_back(desired_vel_marker);
  marker_array.markers.push_back(commanded_yaw_marker);
  marker_array.markers.push_back(current_vel_marker);
  marker_pub_.publish(marker_array);
}

}  // namespace euroc_stage2
