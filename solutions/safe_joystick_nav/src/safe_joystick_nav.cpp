#include <safe_joystick_nav/safe_joystick_nav.h>

namespace safe_joystick_nav {

SafeJoystickNav::SafeJoystickNav(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      vel_max_(1.0),
      acc_max_(2.0),
      yaw_rate_max_(0.6),
      dt_sec_(0.01),
      command_timeout_sec_(0.1),
      min_distance_(1.0),
      max_carrot_distance_(1.0),
      desired_vel_(0, 0, 0),
      desired_yaw_rate_(0) {
  octomap_sub_ =
      nh_.subscribe("octomap_full", 1, &SafeJoystickNav::octomapCallback, this);
  odometry_sub_ =
      nh_.subscribe("odometry", 1, &SafeJoystickNav::odometryCallback, this);
  twist_sub_ = nh_.subscribe("twist_reference", 1, &SafeJoystickNav::twistCallback, this);
  joy_sub_ = nh_.subscribe("joy", 1, &SafeJoystickNav::joyCallback, this);

  command_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1, false);

  world_.reset(new volumetric_mapping::OctomapManager(nh_, nh_private_));

  last_command_time_.fromSec(0);

  // TODO(helenol): params.
  robot_size_ = Eigen::Vector3d(1.0, 1.0, 0.5);
  world_->setRobotSize(robot_size_);
}

void SafeJoystickNav::odometryCallback(const nav_msgs::Odometry& msg) {
  // Publish commands at the same rate as odometry...

  // Check how old our last command was.
  if ((ros::Time::now() - last_command_time_).toSec() > command_timeout_sec_) {
    ROS_INFO_THROTTLE(60, "[Safe Joystick Nav] Command timed out.");
    return;
  }

  // Convert to Eigen message.
  mav_msgs::EigenOdometry odom;
  mav_msgs::eigenOdometryFromMsg(msg, &odom);

  // Create a current state OdometryPoint.
  mav_msgs::EigenTrajectoryPoint current_state;
  current_state.position_W = odom.position_W;
  current_state.orientation_W_B = odom.orientation_W_B;
  current_state.velocity_W = odom.getVelocityWorld();
  // ??? Do we even need this? Is this valid? Assuming only yaw rate is set,
  // w_W = w_B.
  // current_state.angular_velocity_W =
  //    odom.orientation_W_B * odom.angular_velocity_B;

  // Figure out what the next pose should be...
  Eigen::Vector3d current_vel = odom.velocity_B;
  Eigen::Vector3d desired_vel_world = odom.orientation_W_B * desired_vel_;

  ROS_INFO_THROTTLE(
      1, "[Safe Joystick Nav] Desired vel: %f %f %f, current vel: %f %f %f",
      desired_vel_.x(), desired_vel_.y(), desired_vel_.z(), current_vel.x(),
      current_vel.y(), current_vel.z());

  // Enforce acc constraints? Eh whatever.
  mav_msgs::EigenTrajectoryPoint desired_state;
  desired_state.position_W =
      odom.position_W + desired_vel_world * max_carrot_distance_;
  desired_state.orientation_W_B = odom.orientation_W_B;
  desired_state.velocity_W = desired_vel_world;

  // What to do in case this doesn't return true????
  bool success =
      enforceCollisionConstraints(current_state, desired_state, &desired_state);

  if (!success) {
    ROS_WARN("Can not find safe position.");
    return;
  }

  // Now that collisions are resolved, set the yaw rate.
  desired_state.setFromYaw(odom.getYaw() + desired_yaw_rate_);

  geometry_msgs::PoseStamped pose_msg;
  mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(desired_state, &pose_msg);
  pose_msg.header.frame_id = "world";
  command_pub_.publish(pose_msg);
}

void SafeJoystickNav::twistCallback(const geometry_msgs::TwistStamped& msg) {
  // Update the current desired command.
  last_command_time_ = msg.header.stamp;
  desired_vel_ = Eigen::Vector3d(msg.twist.linear.x, msg.twist.linear.y,
                                 msg.twist.linear.z);
  desired_yaw_rate_ = msg.twist.angular.z;
}

void SafeJoystickNav::octomapCallback(const octomap_msgs::Octomap& msg) {
  world_->setOctomapFromMsg(msg);

  // world_->publishAll();
  ROS_INFO_ONCE("[Safe Joystick Nav] Got octomap from message.");
}

void SafeJoystickNav::joyCallback(const sensor_msgs::Joy& msg) {
  // TODO(helenol): move over from joystick-interface in flight manager
  // properly.
  if (!msg.buttons[4]) {
    return;
  }

  Eigen::Vector3d joy_position(0, 0, 0);
  double joy_yaw;
  // Right stick:
  joy_position.x() = deadZone(msg.axes[3]);
  joy_position.y() = deadZone(msg.axes[2]);
  // Left stick:
  joy_position.z() = deadZone(msg.axes[1]);  // Up, down.
  joy_yaw = deadZone(-msg.axes[0]);          // Right, left.

  desired_vel_ = joy_position * vel_max_;
  last_command_time_ = msg.header.stamp;
}

double SafeJoystickNav::deadZone(double axis_pos) const {
  if (std::abs(axis_pos) < 0.2) {
    return 0.0;
  }
  return axis_pos;
}

bool SafeJoystickNav::enforceCollisionConstraints(
    const mav_msgs::EigenTrajectoryPoint& current_point,
    const mav_msgs::EigenTrajectoryPoint& target_point,
    mav_msgs::EigenTrajectoryPoint* safe_point) {
  // First, check if the current state is in collision.
  // Maybe this is reimplemented in getClosestFreePosition, but this function
  // should also consider velocities. :)
  world_->setRobotSize(robot_size_);
  if (world_->checkCollisionWithRobot(current_point.position_W)) {
    // Uh-oh, we're in trouble.
    ROS_WARN("Current state in collision -- this probably shouldn't happen.");
    return false;
  }

  world_->setRobotSize(robot_size_ * 1.5);
  if (world_->checkCollisionWithRobot(target_point.position_W)) {
    // Okay now we HAVE to and CAN do something.
    // Determine closest position that is not in collision along this ray.
    *safe_point = current_point;
    return getClosestFreePosition(current_point.position_W,
                                  target_point.position_W,
                                  &safe_point->position_W);
  }
  return true;

  // TODO(helenol): also forward propagate the velocities somewhat. Make sure
  // can't get INTO collision.
}

bool SafeJoystickNav::getClosestFreePosition(
    const Eigen::Vector3d& current_position,
    const Eigen::Vector3d& target_position, Eigen::Vector3d* safe_position) {
  const int positions_to_test = 10;
  std::vector<Eigen::Vector3d> positions(positions_to_test);

  // Get increment in the correct direction.
  Eigen::Vector3d increment =
      (target_position - current_position) / positions_to_test;

  for (int i = 0; i < positions.size(); ++i) {
    positions[i] = current_position + i * increment;
  }

  size_t collision_index = 0;

  // Returns true if in collision. If this returns false, then the target
  // position is fine anyway. :)
  if (world_->checkPathForCollisionsWithRobot(positions, &collision_index)) {
    if (collision_index <= 0) {
      return false;
    }
    *safe_position = positions[collision_index - 1];
    return true;
  } else {
    *safe_position = target_position;
    return true;
  }
}

}  // namespace safe_joystick_nav
