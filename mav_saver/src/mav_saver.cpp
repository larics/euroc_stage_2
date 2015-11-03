/*
 * mav_saver.cpp
 *
 *  Created on: 18.06.2015
 *      Author: burrimi
 */

#include "mav_saver/mav_saver.hpp"
#include "tf/transform_datatypes.h"
namespace mav_saver {

void VehicleMonitorObserver::Update(
    const std::map<
        std::string,
        std::map<std::string, VehicleMonitorLibrary::ConstraintCheckerOutput> >&
        vehicleStatus) {
  take_control_flag_ = false;
  for (const auto& vehicleStatusMapElement : vehicleStatus) {
    for (const auto& outputMapElement : vehicleStatusMapElement.second) {
      if (outputMapElement.second._constraintSatisfied == false) {
        ROS_WARN_STREAM_THROTTLE(1,
                                 "[" << vehicleStatusMapElement.first
                                     << "] error: " << outputMapElement.first);
        //              << " - Last Valid Position: " <<
        //              outputMapElement.second._lastValidState._linear);
        take_control_flag_ = true;
      }
    }
  }
}

MavSaver::MavSaver(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : vehicle_id_("MAV1"),
      frame_number_(0),
      take_control_flag_(false),
      emergency_button_pressed_prev_(false),
      enable_collision_constraint_(true),
      enable_bounding_volume_constraint_(true),
      enable_attitude_constraint_(true) {
  safety_pose_publisher_.reset(new SafetyPosePublisher(nh, private_nh));

  std::string obstacle_octomap_path;

  private_nh.param("collision_threeshold_in_bounding_sphere_radius",
                   collision_threshold_in_bounding_sphere_radius_,
                   kDefaultCollisionThreesholdInBoundingSphereRadius);
  private_nh.param("max_dist_to_check_collision", max_dist_to_check_collision_,
                   kDefaultMaxDistToCheckCollision);
  private_nh.param("projection_window", projection_window_,
                   kDefaultProjectionWindow);
  private_nh.param("vehicle_radius", vehicle_radius_, kDefaultVehicleRadius);
  private_nh.param("minimum_height_to_check_collision",
                   minimum_height_to_check_collision_,
                   kDefaultMinimumHeightToCheckCollision);

  private_nh.param("obstacle_octomap_path", obstacle_octomap_path,
                   kDefaultObstacleOctomapPath);
  private_nh.param("motion_capture_frequency", motion_capture_frequency_,
                   kDefaultMotionCaptureFrequency);
  private_nh.param("max_roll", max_roll_, kDefaultMaxRoll);
  private_nh.param("max_pitch", max_pitch_, kDefaultMaxPitch);
  private_nh.param("enable_collision_constraint", enable_collision_constraint_,
                   kDefaultEnableCollisionConstraint);
  private_nh.param("enable_bounding_volume_constraint",
                   enable_bounding_volume_constraint_,
                   kDefaultEnableBoundingVolumeConstraint);
  private_nh.param("enable_attitude_constraint", enable_attitude_constraint_,
                   kDefaultEnableAttitudeConstraint);
  private_nh.param("kill_switch_port_name", kill_switch_port_name_,
                   kDefaultKillSwitchPort);
  private_nh.param("kill_switch_check_rate", kill_switch_check_rate_,
                   kDefaultKillSwitchCheckRate);
  private_nh.param("kill_switch_baudrate_", kill_switch_baudrate_,
                   kDefaultKillSwitchBaudrate);
  private_nh.param("kill_switch_wait_time_", kill_switch_wait_time_,
                   kDefaultKillSwitchWaitTime);

  std::string configFilePath = ros::package::getPath("mav_saver");
  std::stringstream ss;
  ss.str("");

  if (obstacle_octomap_path[0] == '/') {
    ss << obstacle_octomap_path;
  } else {
    ss << configFilePath << '/' << obstacle_octomap_path;
  }

  boost::filesystem::path octoMapPath(ss.str());

  vehicle_monitor_observer_.reset(new VehicleMonitorObserver);

  vehicle_monitor_.reset(new VehicleMonitorLibrary::VehicleMonitor(
      octoMapPath, kBoundingBoxCorner1, kBoundingBoxCorner2,
      motion_capture_frequency_));

  vehicle_monitor_->RegisterObserver(vehicle_monitor_observer_);

  registerConstraintCheckers();

  registerVehicle();

  frame_.reset(new VehicleMonitorLibrary::MotionCaptureSystemFrame(0));

  octomap_publisher_ =
      nh.advertise<octomap_msgs::Octomap>("mav_saver_octomap", 1);

  octomap_msg_.header.stamp = ros::Time::now();
  octomap_msg_.header.frame_id = "world";

  octomap_msgs::binaryMapToMsg(*vehicle_monitor_->GetOcTreePtr(), octomap_msg_);

  octomap_publisher_.publish(octomap_msg_);

  // Starting up kill switch
  kill_switch_.reset(new kill_switch_library::KillSwitch(
      kill_switch_check_rate_, kill_switch_wait_time_));
  if (kill_switch_->connect(kill_switch_port_name_, kill_switch_baudrate_)) {
    kill_switch_->start();
    kill_switch_connected_ == true;
  } else {
    kill_switch_connected_ == false;
    ROS_WARN(
        "Failure in connecting the kill switch. Flight will be without kill "
        "switch.");
  }
}

void MavSaver::registerConstraintCheckers() {
  using namespace VehicleMonitorLibrary;

  if (enable_collision_constraint_) {
    std::shared_ptr<CollisionConstraintChecker> collisionChecker =
        std::make_shared<CollisionConstraintChecker>(
            vehicle_monitor_->GetOcTreePtr(),
            vehicle_monitor_->GetEnvironmentBoundingVolume(),
            max_dist_to_check_collision_,  // max distance to check for
                                           // collisions
            collision_threshold_in_bounding_sphere_radius_,  // we consider a
                                                             // collision when
                                                             // the distance is
                                                             // <= 2*radius
            projection_window_, motion_capture_frequency_,
            minimum_height_to_check_collision_);

    vehicle_monitor_->RegisterChecker(collisionChecker);
  }

  if (enable_attitude_constraint_) {
    std::shared_ptr<AttitudeConstraintChecker> attitudeChecker =
        std::make_shared<AttitudeConstraintChecker>(max_roll_, max_pitch_,
                                                    projection_window_,
                                                    motion_capture_frequency_);

    vehicle_monitor_->RegisterChecker(attitudeChecker);
  }

  if (enable_bounding_volume_constraint_) {
    std::shared_ptr<OutOfSpaceConstraintChecker> outOfSpaceChecker(
        new OutOfSpaceConstraintChecker(
            vehicle_monitor_->GetEnvironmentBoundingVolume()));

    vehicle_monitor_->RegisterChecker(outOfSpaceChecker);
  }
}

void MavSaver::registerVehicle() {
  using namespace VehicleMonitorLibrary;

  std::shared_ptr<SimpleVelocityEstimator> velocity_estimator =
      std::make_shared<SimpleVelocityEstimator>(motion_capture_frequency_);

  std::shared_ptr<Vehicle> vehiclePtr = std::make_shared<Vehicle>(
      vehicle_id_, vehicle_radius_, velocity_estimator);
  vehicle_monitor_->RegisterVehicle(vehiclePtr);
}

void MavSaver::setPose(const Eigen::Vector3d& p_W_I,
                       const Eigen::Quaterniond& q_W_I) {
  // vehicle_monitor_SetPose(p_W_I, q_W_I);

  // publishing it once in a while because rviz not always show
  // the octomap if published only at the beginning
  if (frame_number_ % 200 == 0) {
    octomap_publisher_.publish(octomap_msg_);
  }

  VehicleMonitorLibrary::VehicleState vehicle_state;

  // Eigen uses weird convention with range, but otherwise looks ok.
  //  Eigen::Vector3d euler_W_I = q_W_I.toRotationMatrix().eulerAngles(2,1,0);
  //  Eigen::Vector3d angular(euler_W_I.x(),euler_W_I.y(),euler_W_I.z());

  // TODO(burrimi): move to mav_msgs::common.
  tf::Quaternion q(q_W_I.x(), q_W_I.y(), q_W_I.z(), q_W_I.w());
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  //  std::cout << vehicle_id_ << " r " << roll*180/M_PI<< " p " <<
  //  pitch*180/M_PI << " y " << yaw*180/M_PI << std::endl;
  Eigen::Vector3d angular(roll, pitch, yaw);

  vehicle_state.position = p_W_I;
  vehicle_state.orientation = angular;

  checkConstraints(vehicle_state);

  safety_pose_publisher_->SetPose(p_W_I, q_W_I);
}

void MavSaver::setOdometry(const Eigen::Vector3d& p_W_I,
                           const Eigen::Quaterniond& q_W_I,
                           const Eigen::Vector3d& v_W_I,
                           const Eigen::Vector3d& omega_I) {
  // vehicle_monitor_SetPose(p_W_I, q_W_I);

  // publishing it once in a while because rviz not always show
  // the octomap if published only at the beginning
  if (frame_number_ % 200 == 0) {
    octomap_publisher_.publish(octomap_msg_);
  }

  VehicleMonitorLibrary::VehicleState vehicle_state;

  // Eigen uses weird convention with range, but otherwise looks ok.
  //  Eigen::Vector3d euler_W_I = q_W_I.toRotationMatrix().eulerAngles(2,1,0);
  //  Eigen::Vector3d angular(euler_W_I.x(),euler_W_I.y(),euler_W_I.z());

  // TODO(burrimi): move to mav_msgs::common.
  tf::Quaternion q(q_W_I.x(), q_W_I.y(), q_W_I.z(), q_W_I.w());
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  //  std::cout << vehicle_id_ << " r " << roll*180/M_PI<< " p " <<
  //  pitch*180/M_PI << " y " << yaw*180/M_PI << std::endl;
  Eigen::Vector3d orientation(roll, pitch, yaw);

  vehicle_state.position = p_W_I;
  vehicle_state.orientation = orientation;
  vehicle_state.velocity = v_W_I;
  vehicle_state.angular_rate = omega_I;
  vehicle_state.velocity_valid = true;

  checkConstraints(vehicle_state);

  safety_pose_publisher_->SetPose(p_W_I, q_W_I);
}

void MavSaver::checkConstraints(
    const VehicleMonitorLibrary::VehicleState& state) {
  frame_->setFrameNumber(frame_number_);
  frame_->updateFrameElement(vehicle_id_, state);

  // Setting the emergency state by
  // - checking switch state if connected.
  // - assuming no emergency if switch not connected
  bool emergency_button_pressed;
  if (kill_switch_connected_) {
    emergency_button_pressed = kill_switch_->getKillStatus();
    if (emergency_button_pressed) {
      setTakeControlFlag(true);
      ROS_WARN_THROTTLE(
          1, "[MAV_SAVER]: EXTERNAL MAV RESCUE REQUESTED, TAKING OVER !!!");
    }

    // You can release the safety pilot, by releasing the kill switch.
    if (!emergency_button_pressed && emergency_button_pressed_prev_) {
      setTakeControlFlag(false);
      ROS_WARN_THROTTLE(
          1, "[MAV_SAVER]: MAV RELEASED, you're back in control !!!");
    }
  } else {
    emergency_button_pressed = false;
  }
  emergency_button_pressed_prev_ = emergency_button_pressed;

  vehicle_monitor_->Trigger(*frame_, emergency_button_pressed);

  if (vehicle_monitor_observer_->getTakeControlFlag()) {
    setTakeControlFlag(true);
    ROS_WARN_THROTTLE(1, "[MAV_SAVER]: CONSTRAINTS HIT, TAKING OVER !!!");
  }

  safety_pose_publisher_->SetTakeControlFlag(take_control_flag_);

  ++frame_number_;
}

void MavSaver::setTakeControlFlag(double take_control_flag) {
  take_control_flag_ = take_control_flag;
}
}
