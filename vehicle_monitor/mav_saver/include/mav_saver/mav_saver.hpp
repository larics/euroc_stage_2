/*
 * mav_saver.hpp
 *
 *  Created on: 18.06.2015
 *      Author: burrimi
 */

#ifndef INCLUDE_MAV_SAVER_HPP_
#define INCLUDE_MAV_SAVER_HPP_

//#include "mav_saver/vehicle_monitor.hpp"
#include "mav_saver/safety_pose_publisher.hpp"



#include <Eigen/Eigen>

#include <vehicle_monitor_library/VehicleMonitor.hpp>
#include <vehicle_monitor_library/VehicleMonitorObserver.hpp>
#include <vehicle_monitor_library/Vehicle.hpp>
#include <vehicle_monitor_library/SimpleVelocityEstimator.hpp>
#include <vehicle_monitor_library/AttitudeConstraintChecker.hpp>
#include <vehicle_monitor_library/CollisionConstraintChecker.hpp>
#include <vehicle_monitor_library/OutOfSpaceConstraintChecker.hpp>
#include <vehicle_monitor_library/MotionCaptureSystemFrame.hpp>

#include <kill_switch_library/kill_switch.h>

#include <random>
#include <chrono>
#include <thread>
#include <string>

#include <iostream>

#include <octomap/octomap_types.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include "yaml-cpp/yaml.h"

#include <boost/filesystem.hpp>

#include "ros/ros.h"
#include "ros/package.h"
#include "ros/publisher.h"
#include "geometry_msgs/PointStamped.h"


namespace mav_saver {



class VehicleMonitorObserver : public VehicleMonitorLibrary::VehicleMonitorObserverBase {
 public:
  VehicleMonitorObserver():take_control_flag_(false) { };
  virtual ~VehicleMonitorObserver() { };

  virtual void Update(const std::map<std::string, std::map<std::string, VehicleMonitorLibrary::ConstraintCheckerOutput> >& vehicleStatus);

  bool getTakeControlFlag() const {
    return take_control_flag_;
  }

 private:
  bool take_control_flag_;
};

// Default values
constexpr int kDefaultMotionCaptureFrequency = 100;
constexpr double kDefaultCollisionThreesholdInBoundingSphereRadius = 1.5;
constexpr double kDefaultMaxDistToCheckCollision = 2.0;
constexpr int kDefaultProjectionWindow = 20;
constexpr double kDefaultVehicleRadius = 0.6;
constexpr double kDefaultMaxRoll = 20.0/180.0*M_PI;
constexpr double kDefaultMaxPitch = 20.0/180.0*M_PI;

const std::string kDefaultObstacleOctomapPath = "res/LeoC6.bt";
const Eigen::Vector3d kBoundingBoxCorner1(-5.0, -5.0, -1.0);
const Eigen::Vector3d kBoundingBoxCorner2(5.0, 5.0, 5.0);

const std::string kDefaultKillSwitchPort = "/dev/ttyUSB1" ;
constexpr double kDefaultKillSwitchCheckRate = 10.0;
constexpr int kDefaultKillSwitchBaudrate = 9600;
constexpr double kDefaultKillSwitchWaitTime = 2.0;

class MavSaver {
 public:
  MavSaver(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

  void SetPose(const Eigen::Vector3d& p_W_I, const Eigen::Quaterniond& q_W_I);

  void SetTakeControlFlag(double take_control_flag);

 private:

  void RegisterConstraintCheckers();

  void RegisterVehicle();

  std::shared_ptr<VehicleMonitorLibrary::VehicleMonitor> vehicle_monitor_;

  std::shared_ptr<VehicleMonitorObserver> vehicle_monitor_observer_;

  std::shared_ptr<SafetyPosePublisher> safety_pose_publisher_;

  int motion_capture_frequency_;

  ros::Publisher octomap_publisher_;

  octomap_msgs::Octomap octomap_msg_;

  double collision_threshold_in_bounding_sphere_radius_;
  double max_dist_to_check_collision_;
  int projection_window_;
  double vehicle_radius_;

  std::string vehicle_id_;

  std::shared_ptr<VehicleMonitorLibrary::MotionCaptureSystemFrame> frame_;

  int frame_number_;

  double max_roll_;
  double max_pitch_;

  bool take_control_flag_;

  std::shared_ptr<kill_switch_library::KillSwitch> kill_switch_;
  std::string kill_switch_port_name_;
  double kill_switch_check_rate_;
  int kill_switch_baudrate_;
  double kill_switch_wait_time_;
  bool kill_switch_connected_;
  bool emergency_button_pressed_prev_;
};

}


#endif /* INCLUDE_MAV_SAVER_HPP_ */
