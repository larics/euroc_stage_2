/*
 * mav_saver.hpp
 *
 *  Created on: 18.06.2015
 *      Author: burrimi
 */

#ifndef INCLUDE_MAV_SAVER_HPP_
#define INCLUDE_MAV_SAVER_HPP_

#include "mav_saver/safety_pose_publisher.hpp"

#include <Eigen/Eigen>

#include <vehicle_monitor_library/AttitudeConstraintChecker.hpp>
#include <vehicle_monitor_library/CollisionConstraintChecker.hpp>
#include <vehicle_monitor_library/MotionCaptureSystemFrame.hpp>
#include <vehicle_monitor_library/OutOfSpaceConstraintChecker.hpp>
#include <vehicle_monitor_library/SimpleVelocityEstimator.hpp>
#include <vehicle_monitor_library/Vehicle.hpp>
#include <vehicle_monitor_library/VehicleMonitor.hpp>
#include <vehicle_monitor_library/VehicleMonitorObserver.hpp>
#include <vehicle_monitor_library/VelocityConstraintChecker.hpp>

#include <kill_switch_library/kill_switch.h>

#include <chrono>
#include <random>
#include <string>
#include <thread>

#include <iostream>

#include <octomap/octomap_types.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include "yaml-cpp/yaml.h"

#include <boost/filesystem.hpp>

#include "geometry_msgs/PointStamped.h"
#include "ros/package.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"

namespace mav_saver {

class VehicleMonitorObserver
    : public VehicleMonitorLibrary::VehicleMonitorObserverBase {
 public:
  VehicleMonitorObserver(ros::NodeHandle* const nh);
  virtual ~VehicleMonitorObserver(){};

  virtual void Update(const std::map<
      std::string,
      std::map<std::string, VehicleMonitorLibrary::ConstraintCheckerOutput> >&
                          vehicleStatus);

  bool getTakeControlFlag() const { return take_control_flag_; }

 private:
  bool take_control_flag_;
};

// Default values
constexpr int kDefaultMotionCaptureFrequency = 100;
constexpr double kDefaultCollisionThreesholdDistance = 0.3;
constexpr int kDefaultProjectionWindow = 50;
constexpr double kDefaultVehicleRadius = 0.4;
constexpr double kDefaultVehicleHeight = 0.24;
constexpr double kDefaultMinimumHeightToCheckCollision = 0.7;
constexpr double kDefaultMaxRoll = 30.0 / 180.0 * M_PI;
constexpr double kDefaultMaxPitch = 30.0 / 180.0 * M_PI;

constexpr double kDefaultAcceptableViolationDuration = 0.00;
constexpr double kDefaultMaxVelocity = 2;

const Eigen::Vector3d kBoundingBoxCorner1(-5.0, -5.0, -1.0);
const Eigen::Vector3d kBoundingBoxCorner2(5.0, 5.0, 5.0);

constexpr bool kDefaultEnableCollisionConstraint(true);
constexpr bool kDefaultEnableBoundingVolumeConstraint(true);
constexpr bool kDefaultEnableAttitudeConstraint(true);
constexpr bool kDefaultEnableVelocityConstraint(true);

const std::string kDefaultKillSwitchPort = "/dev/ttyUSB1";
constexpr double kDefaultKillSwitchCheckRate = 10.0;
constexpr int kDefaultKillSwitchBaudrate = 9600;
constexpr double kDefaultKillSwitchWaitTime = 2.0;

class MavSaver {
 public:
  MavSaver(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

  void setOctomap(const octomap_msgs::OctomapConstPtr& msg);
  void setPose(const Eigen::Vector3d& p_W_I, const Eigen::Quaterniond& q_W_I);
  void setOdometry(const Eigen::Vector3d& p_W_I,
                   const Eigen::Quaterniond& q_W_I,
                   const Eigen::Vector3d& v_W_I,
                   const Eigen::Vector3d& omega_I);

  void checkConstraints(const VehicleMonitorLibrary::VehicleState& state);

  void setTakeControlFlag(bool take_control_flag);

  bool getTakeControlFlag(void);

 private:
  enum TextType { INFO, WARN, ERR };

  void registerConstraintCheckers();

  void registerVehicle();

  void setupRvizMarker(ros::NodeHandle* const nh);

  void setRvizMarkerPosition(const Eigen::Vector3d& pos,
                             const Eigen::Quaterniond& rot);

  void setRvizMarkerPosition(const Eigen::Vector3d& pos,
                             const Eigen::Quaterniond& rot,
                             const Eigen::Vector3d& vel);

  void setRvizMarkerText(const std::string& text, TextType type);

  void updateRviz(void);

  std::shared_ptr<VehicleMonitorLibrary::VehicleMonitor> vehicle_monitor_;

  std::shared_ptr<VehicleMonitorObserver> vehicle_monitor_observer_;

  std::shared_ptr<SafetyPosePublisher> safety_pose_publisher_;

  int motion_capture_frequency_;

  double collision_threshold_distance_;

  int projection_window_;
  double vehicle_radius_;
  double vehicle_height_;
  double minimum_height_to_check_collision_;

  double acceptable_violation_duration_;
  ros::Time violation_time_;

  std::string vehicle_id_;

  std::shared_ptr<VehicleMonitorLibrary::MotionCaptureSystemFrame> frame_;

  int frame_number_;

  double max_roll_;
  double max_pitch_;
  double max_velocity_;

  ros::Publisher constraints_violated_publisher_;
  bool take_control_flag_;

  std::shared_ptr<kill_switch_library::KillSwitch> kill_switch_;
  std::string kill_switch_port_name_;
  double kill_switch_check_rate_;
  int kill_switch_baudrate_;
  double kill_switch_wait_time_;
  bool kill_switch_connected_;
  bool emergency_button_pressed_prev_;

  bool enable_collision_constraint_;
  bool enable_bounding_volume_constraint_;
  bool enable_attitude_constraint_;
  bool enable_velocity_constraint_;

  std::shared_ptr<VehicleMonitorLibrary::OctreeHolder> octree_ptr_;

  ros::Publisher viz_publisher_;

  visualization_msgs::MarkerArray markers_mav_bbox_;
};
}

#endif /* INCLUDE_MAV_SAVER_HPP_ */
