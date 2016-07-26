#include <octomap_ros/conversions.h>
#include <visualization_msgs/MarkerArray.h>

#include <euroc_stage2/trajectory_interface.h>
#include <euroc_stage2/csv_parser.h>

#include <euroc_stage2/task4_eval.h>

namespace euroc_stage2 {

Task4Eval::Task4Eval(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private)
    : EvalBase(nh, nh_private, "task4"),
      nh_(nh),
      nh_private_(nh_private),
      dt_sec_(0.05),
      octomap_min_z_(0.4),
      octomap_max_z_(1.9),
      received_twist_ref_(false),
      last_vel_command_(Eigen::Vector3d::Zero()),
      last_yaw_rate_command_(0.0) {
  odometry_sub_ =
      nh_.subscribe("odometry", 1, &Task4Eval::odometryCallback, this);

  // Twist reference from the task 4 server.
  twist_sub_ =
      nh_.subscribe("twist_reference", 1, &Task4Eval::twistCallback, this);
  octomap_sub_ =
      nh_.subscribe("octomap", 1, &Task4Eval::setOctomapFromMsg, this);

  nh_private_.param("octomap_file", octomap_file_, octomap_file_);
  nh_private_.param("octomap_min_z", octomap_min_z_, octomap_min_z_);
  nh_private_.param("octomap_max_z", octomap_max_z_, octomap_max_z_);

  loadOctomapFromFile();

  results_writer_
      << "#tstamp_ns, pos_x, pos_y, pos_z, yaw, vel_x, vel_y, vel_z, "
         "yaw_rate, cvel_x, cvel_y, cvel_z, cyaw_rate, dist\n";
}

Task4Eval::~Task4Eval() {}

void Task4Eval::setOctomapFromMsg(const octomap_msgs::Octomap& msg) {
  if (octree_ != nullptr) {
    ROS_INFO_ONCE("Already received octomap, not resetting.");
  }

  if (msg.binary) {
    octree_.reset(
        dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(msg)));
  } else {
    octree_.reset(
        dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(msg)));
  }
  ROS_INFO_ONCE("Initializing octree from message.");
  initializeDistanceMap();
}

void Task4Eval::odometryCallback(const nav_msgs::Odometry& msg) {
  double message_dt = (msg.header.stamp - last_msg_time_).toSec();
  if (message_dt < dt_sec_) {
    return;
  } else {
    last_msg_time_ = msg.header.stamp;
  }

  // Convert msg to Eigen...
  mav_msgs::EigenOdometry odom;
  mav_msgs::eigenOdometryFromMsg(msg, &odom);

  // Before computing new command, calculate error from last one and output.
  if (received_twist_ref_ &&
      std::abs((last_ref_time_ - last_msg_time_).toSec()) <
          kReferenceTimeoutSec) {
    calculateScoreAndOutput(odom);
  }
}

void Task4Eval::twistCallback(const geometry_msgs::TwistStamped& twist_msg) {
  // Save the last set of commands for comparison on next tick.
  last_vel_command_ =
      Eigen::Vector3d(twist_msg.twist.linear.x, twist_msg.twist.linear.y,
                      twist_msg.twist.linear.z);
  last_yaw_rate_command_ = twist_msg.twist.angular.z;
  received_twist_ref_ = true;
  last_ref_time_ = twist_msg.header.stamp;
}

void Task4Eval::calculateScoreAndOutput(const mav_msgs::EigenOdometry& odom) {
  // File format:
  // timestamp, pos [x, y, z, yaw], actual vel BODY [x, y, z, yaw_rate],
  // commanded vel BODY [x, y, z, yaw_rate], distance of CENTER from obstacle
  // [m]

  // Also output score?
  // Score should only consider velocity when further than X from obstacle,
  // otherwise only obstacle distance.
  const std::string sep(", ");
  double center_distance_from_obstacle = 0.0;

  if (distance_map_) {
    octomap::point3d p(odom.position_W.x(), odom.position_W.y(),
                       odom.position_W.z());
    center_distance_from_obstacle = distance_map_->getDistance(p);
  }

  // TODO(helenol): switch to common result outputter.
  results_writer_ << odom.timestamp_ns << sep << odom.position_W.x() << sep
                  << odom.position_W.y() << sep << odom.position_W.z() << sep
                  << odom.getYaw() << sep << odom.velocity_B.x() << sep
                  << odom.velocity_B.y() << sep << odom.velocity_B.z() << sep
                  << odom.getYawRate() << sep << last_vel_command_.x() << sep
                  << last_vel_command_.y() << sep << last_vel_command_.z()
                  << sep << last_yaw_rate_command_ << sep
                  << center_distance_from_obstacle << "\n";
}

void Task4Eval::loadOctomapFromFile() {
  // First, load the actual tree.
  if (octomap_file_.empty()) {
    ROS_WARN(
        "No octomap loaded from file, unable to compute distances until "
        "octomap message received.");
    return;
  }

  octree_.reset(new octomap::OcTree(octomap_file_));

  initializeDistanceMap();
}

void Task4Eval::initializeDistanceMap() {
  // Then set up everything to create a distance map (positive only) of the
  // entire map. This will allow us to evaluate distances to objects.
  const bool unknown_as_occupied = false;

  double x, y, z;
  // Fill in metric min and max of the distance map from the full octomap.
  octree_->getMetricMin(x, y, z);
  octomap::point3d min(x, y, octomap_min_z_);
  octree_->getMetricMax(x, y, z);
  octomap::point3d max(x, y, octomap_max_z_);

  distance_map_.reset(new DynamicEDTOctomap(
      kOctomapMaxDistMeters, octree_.get(), min, max, unknown_as_occupied));

  // Compute the full map.
  distance_map_->update();
  ROS_INFO_ONCE("Computed full distance map.");
}

}  // namespace euroc_stage2
