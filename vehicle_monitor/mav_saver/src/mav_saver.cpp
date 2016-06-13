/*
 * mav_saver.cpp
 *
 *  Created on: 18.06.2015
 *      Author: burrimi
 */

#include "mav_saver/mav_saver.hpp"
#include "tf/transform_datatypes.h"
namespace mav_saver {

enum MarkerIdx { MAV = 0, BBOX, TEXT, FUTURE_BBOX, MARKER_IDX_LENGTH };

VehicleMonitorObserver::VehicleMonitorObserver(ros::NodeHandle* const nh)
    : take_control_flag_(false) {}
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

void MavSaver::setupRvizMarker(ros::NodeHandle* const nh) {
  visualization_msgs::Marker marker_mav;
  marker_mav.header.frame_id = "world";
  marker_mav.id = MAV;
  marker_mav.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker_mav.action = visualization_msgs::Marker::ADD;
  marker_mav.scale.x = 1.0;
  marker_mav.scale.y = 1.0;
  marker_mav.scale.z = 1.0;
  marker_mav.color.a = 1.0;
  marker_mav.mesh_resource = "package://mav_saver/meshes/euroc_hex.dae";
  marker_mav.mesh_use_embedded_materials = true;

  markers_mav_bbox_.markers.push_back(marker_mav);

  visualization_msgs::Marker marker_bbox;
  marker_bbox.header.frame_id = "world";
  marker_bbox.id = BBOX;
  marker_bbox.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker_bbox.action = visualization_msgs::Marker::ADD;
  marker_bbox.scale.x = 1.0;
  marker_bbox.scale.y = 1.0;
  marker_bbox.scale.z = 1.0;
  marker_bbox.color.a = 0.2;
  marker_bbox.color.r = 1.0;
  marker_bbox.color.g = 0.0;
  marker_bbox.color.b = 0.0;
  marker_bbox.mesh_resource = "package://mav_saver/meshes/bbox.dae";

  markers_mav_bbox_.markers.push_back(marker_bbox);

  visualization_msgs::Marker marker_text;
  marker_text.header.frame_id = "world";
  marker_text.id = TEXT;
  marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_text.action = visualization_msgs::Marker::ADD;
  marker_text.scale.z = 0.2;
  marker_text.color.a = 1.0;
  marker_text.color.r = 1.0;
  marker_text.color.g = 1.0;
  marker_text.color.b = 1.0;
  marker_text.text = "MAV Saver running...";

  markers_mav_bbox_.markers.push_back(marker_text);

  visualization_msgs::Marker marker_future_bbox;
  marker_future_bbox.header.frame_id = "world";
  marker_future_bbox.id = FUTURE_BBOX;
  marker_future_bbox.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker_future_bbox.action = visualization_msgs::Marker::ADD;
  marker_future_bbox.scale.x = 1.0;
  marker_future_bbox.scale.y = 1.0;
  marker_future_bbox.scale.z = 1.0;
  marker_future_bbox.color.a = 0.0;  // initially invisible
  marker_future_bbox.color.r = 0.5;
  marker_future_bbox.color.g = 0.0;
  marker_future_bbox.color.b = 0.0;
  marker_future_bbox.mesh_resource = "package://mav_saver/meshes/bbox.dae";

  markers_mav_bbox_.markers.push_back(marker_future_bbox);

  viz_publisher_ = nh->advertise<visualization_msgs::MarkerArray>(
      "mav_saver_status_markers", 0);
}

void MavSaver::setRvizMarkerPosition(const Eigen::Vector3d& pos,
                                     const Eigen::Quaterniond& rot) {
  markers_mav_bbox_.markers[MAV].pose.orientation.x = rot.x();
  markers_mav_bbox_.markers[MAV].pose.orientation.y = rot.y();
  markers_mav_bbox_.markers[MAV].pose.orientation.z = rot.z();
  markers_mav_bbox_.markers[MAV].pose.orientation.w = rot.w();

  for (size_t i = 0; i < MARKER_IDX_LENGTH; ++i) {
    markers_mav_bbox_.markers[i].pose.position.x = pos[0];
    markers_mav_bbox_.markers[i].pose.position.y = pos[1];
    markers_mav_bbox_.markers[i].pose.position.z = pos[2];
  }

  markers_mav_bbox_.markers[TEXT].pose.position.z += 0.5;

  // no vel info so hide future pose
  markers_mav_bbox_.markers[FUTURE_BBOX].color.a = 0.0;
}

void MavSaver::setRvizMarkerPosition(const Eigen::Vector3d& pos,
                                     const Eigen::Quaterniond& rot,
                                     const Eigen::Vector3d& vel) {
  setRvizMarkerPosition(pos, rot);

  // future pose
  double delta = static_cast<double>(projection_window_) /
                 static_cast<double>(motion_capture_frequency_);
  markers_mav_bbox_.markers[FUTURE_BBOX].color.a = 0.4;
  markers_mav_bbox_.markers[FUTURE_BBOX].pose.position.x += delta * vel.x();
  markers_mav_bbox_.markers[FUTURE_BBOX].pose.position.y += delta * vel.y();
  markers_mav_bbox_.markers[FUTURE_BBOX].pose.position.z += delta * vel.z();
}

void MavSaver::setRvizMarkerText(const std::string& text, TextType type) {
  markers_mav_bbox_.markers[TEXT].text = text.c_str();

  switch (type) {
    case (WARN):
      markers_mav_bbox_.markers[TEXT].color.r = 1.0;
      markers_mav_bbox_.markers[TEXT].color.g = 1.0;
      markers_mav_bbox_.markers[TEXT].color.b = 0.0;
      break;
    case (ERR):
      markers_mav_bbox_.markers[TEXT].color.r = 1.0;
      markers_mav_bbox_.markers[TEXT].color.g = 0.0;
      markers_mav_bbox_.markers[TEXT].color.b = 0.0;
      break;
    default:
      markers_mav_bbox_.markers[TEXT].color.r = 1.0;
      markers_mav_bbox_.markers[TEXT].color.g = 1.0;
      markers_mav_bbox_.markers[TEXT].color.b = 1.0;
  }
}

void MavSaver::updateRviz(void) { viz_publisher_.publish(markers_mav_bbox_); }

MavSaver::MavSaver(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : vehicle_id_("MAV1"),
      frame_number_(0),
      take_control_flag_(false),
      emergency_button_pressed_prev_(false),
      enable_collision_constraint_(true),
      enable_bounding_volume_constraint_(true),
      enable_attitude_constraint_(true),
      octree_ptr_(new VehicleMonitorLibrary::OctreeHolder) {
  safety_pose_publisher_.reset(new SafetyPosePublisher(nh, private_nh));

  constraints_violated_publisher_ =
      nh.advertise<std_msgs::Bool>("saver_constraints_violated", 0);

  std::vector<std::string> obstacle_octomap_paths;

  private_nh.param("collision_threshold_distance",
                   collision_threshold_distance_,
                   kDefaultCollisionThreesholdDistance);
  private_nh.param("projection_window", projection_window_,
                   kDefaultProjectionWindow);
  private_nh.param("vehicle_radius", vehicle_radius_, kDefaultVehicleRadius);
  private_nh.param("vehicle_height", vehicle_height_, kDefaultVehicleHeight);
  private_nh.param("minimum_height_to_check_collision",
                   minimum_height_to_check_collision_,
                   kDefaultMinimumHeightToCheckCollision);

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

  setupRvizMarker(&nh);

  vehicle_monitor_observer_.reset(new VehicleMonitorObserver(&nh));

  vehicle_monitor_.reset(new VehicleMonitorLibrary::VehicleMonitor(
      octree_ptr_, kBoundingBoxCorner1, kBoundingBoxCorner2,
      motion_capture_frequency_));

  vehicle_monitor_->registerObserver(vehicle_monitor_observer_);

  registerConstraintCheckers();

  registerVehicle();

  frame_.reset(new VehicleMonitorLibrary::MotionCaptureSystemFrame(0));

  // Starting up kill switch
  kill_switch_.reset(new kill_switch_library::KillSwitch(
      kill_switch_check_rate_, kill_switch_wait_time_));
  if (kill_switch_->connect(kill_switch_port_name_, kill_switch_baudrate_)) {
    kill_switch_->start();
    kill_switch_connected_ = true;
  } else {
    kill_switch_connected_ = false;
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
            vehicle_monitor_->getOcTreePtr(), collision_threshold_distance_,
            vehicle_height_, vehicle_radius_, projection_window_,
            motion_capture_frequency_, minimum_height_to_check_collision_);

    vehicle_monitor_->registerChecker(collisionChecker);
  }

  if (enable_attitude_constraint_) {
    std::shared_ptr<AttitudeConstraintChecker> attitudeChecker =
        std::make_shared<AttitudeConstraintChecker>(max_roll_, max_pitch_,
                                                    projection_window_,
                                                    motion_capture_frequency_);

    vehicle_monitor_->registerChecker(attitudeChecker);
  }

  if (enable_bounding_volume_constraint_) {
    std::shared_ptr<OutOfSpaceConstraintChecker> outOfSpaceChecker(
        new OutOfSpaceConstraintChecker(
            vehicle_monitor_->getEnvironmentBoundingVolume()));

    vehicle_monitor_->registerChecker(outOfSpaceChecker);
  }
}

void MavSaver::registerVehicle() {
  using namespace VehicleMonitorLibrary;

  std::shared_ptr<SimpleVelocityEstimator> velocity_estimator =
      std::make_shared<SimpleVelocityEstimator>(motion_capture_frequency_);

  std::shared_ptr<Vehicle> vehiclePtr = std::make_shared<Vehicle>(
      vehicle_id_, vehicle_radius_, velocity_estimator);

  vehicle_monitor_->registerVehicle(vehiclePtr);
}

void MavSaver::setOctomap(const octomap_msgs::OctomapConstPtr& msg) {
  octree_ptr_->updateOctree(msg);
}

void MavSaver::setPose(const Eigen::Vector3d& p_W_I,
                       const Eigen::Quaterniond& q_W_I) {
  // vehicle_monitor_SetPose(p_W_I, q_W_I);

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
  setRvizMarkerPosition(p_W_I, q_W_I);
  updateRviz();
}

void MavSaver::setOdometry(const Eigen::Vector3d& p_W_I,
                           const Eigen::Quaterniond& q_W_I,
                           const Eigen::Vector3d& v_W_I,
                           const Eigen::Vector3d& omega_I) {
  // vehicle_monitor_SetPose(p_W_I, q_W_I);

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
  setRvizMarkerPosition(p_W_I, q_W_I, v_W_I);
  updateRviz();
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
      setRvizMarkerText("KILL SWITCH HIT, SAVER ACTIVATED", ERR);
      updateRviz();
    }

    // You can release the safety pilot, by releasing the kill switch.
    if (!emergency_button_pressed && emergency_button_pressed_prev_) {
      setTakeControlFlag(false);
      vehicle_monitor_->resetAllChecker();
      ROS_WARN_THROTTLE(
          1, "[MAV_SAVER]: MAV RELEASED, you're back in control !!!");
      setRvizMarkerText("MAV Saver running...", INFO);
      updateRviz();
    }
  } else {
    emergency_button_pressed = false;
  }
  emergency_button_pressed_prev_ = emergency_button_pressed;

  vehicle_monitor_->trigger(*frame_, emergency_button_pressed);

  if (vehicle_monitor_observer_->getTakeControlFlag()) {
    setTakeControlFlag(true);
    std::string text = "[MAV_SAVER]: CONSTRAINTS HIT, TAKING OVER !!!";
    ROS_WARN_THROTTLE(1, text.c_str());
    setRvizMarkerText("CONSTRAINT HIT, SAVER ACTIVATED", ERR);
    updateRviz();
  }

  safety_pose_publisher_->SetTakeControlFlag(take_control_flag_);
  std_msgs::Bool take_control_flag_msg;
  take_control_flag_msg.data = take_control_flag_;
  constraints_violated_publisher_.publish(take_control_flag_msg);

  ++frame_number_;
}

void MavSaver::setTakeControlFlag(bool take_control_flag) {
  take_control_flag_ = take_control_flag;
}

bool MavSaver::getTakeControlFlag(void) { return take_control_flag_; }
}
