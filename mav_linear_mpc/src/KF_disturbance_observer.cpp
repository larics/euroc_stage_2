/*
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 *
 * This code can be used only for academic, non-commercial use.
 * This code cannot be redistributed under any license, open source or otherwise.
 *
 * CVXGEN license: http://cvxgen.com/docs/license.html
 * FORCES license: http://forces.ethz.ch
 *
 */

#include <mav_linear_mpc/KF_disturbance_observer.h>

namespace mav_control {

KFDisturbanceObserver::KFDisturbanceObserver(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      observer_nh_(private_nh, "observer"),
      gravity_(9.81),
      initialized_(false),
      calibrate_(false),
      F_(kStateSize, kStateSize),
      H_(kMeasurementSize, kStateSize)
{
  this->Initialize();
}

bool KFDisturbanceObserver::StartCalibrationCallback(std_srvs::Empty::Request& req,
                                                     std_srvs::Empty::Response& res)
{
  if (initialized_) {
    calibrate_ = true;
    forces_offset_.setZero();
    moments_offset_.setZero();
    calibration_counter_ = 0;
    start_calibration_time_ = ros::Time::now();
    return true;
  }
  ROS_WARN("KF Calibration Failed...");
  return false;
}

void KFDisturbanceObserver::Initialize()
{

  printf("Start initializing KF\n");

  service_ = observer_nh_.advertiseService("StartCalibrateKF",
                                           &KFDisturbanceObserver::StartCalibrationCallback, this);

  observer_state_pub_ = observer_nh_.advertise<mav_linear_mpc::ObserverState>("observer_state", 10);

  std::vector<double> temporary_F, temporary_Q, temporary_P, temporary_R, temporary_drag;
  std::vector<double> temporary_external_forces_limit, temporary_external_moments_limit;
  std::vector<double> temporary_omega_limit;

  double calibration_duration;
  if (!observer_nh_.getParam("calibration_duration", calibration_duration))
    ROS_ERROR("calibration_duration in KF are not loaded from ros parameter server");
  calibration_duration_ = ros::Duration(calibration_duration);

  if (!observer_nh_.getParam("drag_coefficients", temporary_drag))
    ROS_ERROR("Drag Coefficients in KF are not loaded from ros parameter server");

  if (!observer_nh_.getParam("roll_omega", roll_omega_))
    ROS_ERROR("roll_omega in KF is not loaded from ros parameter server");
  if (!observer_nh_.getParam("roll_damping", roll_damping_))
    ROS_ERROR("roll_damping in KF is not loaded from ros parameter server");
  if (!observer_nh_.getParam("roll_gain", roll_gain_))
    ROS_ERROR("roll_gain in KF is not loaded from ros parameter server");

  if (!observer_nh_.getParam("pitch_omega", pitch_omega_))
    ROS_ERROR("pitch_omega in KF is not loaded from ros parameter server");
  if (!observer_nh_.getParam("pitch_damping", pitch_damping_))
    ROS_ERROR("pitch_damping in KF is not loaded from ros parameter server");
  if (!observer_nh_.getParam("pitch_gain", pitch_gain_))
    ROS_ERROR("pitch_gain in KF is not loaded from ros parameter server");

  if (!observer_nh_.getParam("yaw_omega", yaw_omega_))
    ROS_ERROR("yaw_omega in KF is not loaded from ros parameter server");
  if (!observer_nh_.getParam("yaw_damping", yaw_damping_))
    ROS_ERROR("yaw_damping in KF is not loaded from ros parameter server");
  if (!observer_nh_.getParam("yaw_gain", yaw_gain_))
    ROS_ERROR("yaw_gain in KF is not loaded from ros parameter server");

  if (!observer_nh_.getParam("F", temporary_F))
    ROS_ERROR("F in KF is not loaded from ros parameter server");
  if (!observer_nh_.getParam("Q", temporary_Q))
    ROS_ERROR("Q in KF is not loaded from ros parameter server");
  if (!observer_nh_.getParam("P_0", temporary_P))
    ROS_ERROR("P_0 in KF is not loaded from ros parameter server");
  if (!observer_nh_.getParam("R", temporary_R))
    ROS_ERROR("R in KF is not loaded from ros parameter server");

  if (!observer_nh_.getParam("external_forces_limit", temporary_external_forces_limit))
    ROS_ERROR("external_forces_limit in KF is not loaded from ros parameter server");
  if (!observer_nh_.getParam("external_moments_limit", temporary_external_moments_limit))
    ROS_ERROR("external_moments_limit in KF is not loaded from ros parameter server");
  if (!observer_nh_.getParam("omega_limit", temporary_omega_limit))
    ROS_ERROR("omega_limit in KF is not loaded from ros parameter server");

  ROS_INFO("KF parameters loaded successfully");

  state_covariance_.setZero();
  process_noise_covariance_.setZero();
  measurement_covariance_.setZero();

  // First 9x9 (=measurement size) block is identity, rest is zero.
  H_.reserve(kMeasurementSize);
  for (int i = 0; i < kMeasurementSize; ++i) {
    H_.insert(i, i) = 1.0;
  }

  for (int i = 0; i < kStateSize; i++) {
    state_covariance_(i, i) = temporary_P.at(i);
    process_noise_covariance_(i) = temporary_Q.at(i);
  }

  for (int i = 0; i < kMeasurementSize; i++) {
    measurement_covariance_(i, i) = temporary_R.at(i);
  }

  Eigen::Map<Eigen::MatrixXd> F_map(temporary_F.data(), kStateSize, kStateSize);
  Eigen::Map<Eigen::Vector3d> external_forces_limit_map(temporary_external_forces_limit.data(), 3,
                                                        1);
  Eigen::Map<Eigen::Vector3d> external_moments_limit_map(temporary_external_moments_limit.data(), 3,
                                                         1);
  Eigen::Map<Eigen::Vector3d> omega_limit_map(temporary_omega_limit.data(), 3, 1);

  external_forces_limit_ = external_forces_limit_map;
  external_moments_limit_ = external_moments_limit_map;
  omega_limit_ = omega_limit_map;

  // Insert non-zero elements from F_map into F_
  constexpr double double_tol = 1e-12;
  for (size_t row = 0; row < kStateSize; ++row) {
    for (size_t col = 0; col < kStateSize; ++col) {
      if (std::abs(F_map(row, col)) > double_tol) {
        F_.insert(row, col) = F_map(row, col);
      }
    }
  }
  F_.makeCompressed();

  drag_coefficients_matrix_.setZero();
  for (int i = 0; i < 3; i++) {
    drag_coefficients_matrix_(i, i) = temporary_drag.at(i);
  }

  state_.setZero();
  predicted_state_.setZero();
  forces_offset_.setZero();
  moments_offset_.setZero();

  initialized_ = true;

  ROS_INFO("Kalman Filter Initialized!");

}

void KFDisturbanceObserver::FeedPositionMeasurement(const Eigen::Vector3d& position)
{
  this->measurements_(0) = position(0);
  this->measurements_(1) = position(1);
  this->measurements_(2) = position(2);
}

void KFDisturbanceObserver::FeedVelocityMeasurement(const Eigen::Vector3d& velocity)
{
  this->measurements_(3) = velocity(0);
  this->measurements_(4) = velocity(1);
  this->measurements_(5) = velocity(2);
}

void KFDisturbanceObserver::FeedRotationMatrix(const Eigen::Matrix3d& rotation_matrix)
{
  this->rotation_matrix_ = rotation_matrix;
  this->measurements_(6) = atan2((double) rotation_matrix(2, 1), (double) rotation_matrix(2, 2));
  this->measurements_(7) = -asin((double) rotation_matrix(2, 0));
  this->measurements_(8) = atan2((double) rotation_matrix(1, 0), (double) rotation_matrix(0, 0));
}

void KFDisturbanceObserver::FeedAttitudeCommand(const Eigen::Vector4d& roll_pitch_yaw_thrust_cmd)
{
  this->roll_pitch_yaw_thrust_cmd_ = roll_pitch_yaw_thrust_cmd;
}

void KFDisturbanceObserver::Reset(const Eigen::Vector3d& initial_position,
                                  const Eigen::Vector3d& initial_velocity,
                                  const Eigen::Vector3d& initial_attitude,
                                  const Eigen::Vector3d& initial_angular_rate,
                                  const Eigen::Vector3d& initial_external_forces,
                                  const Eigen::Vector3d& initial_external_moments)
{
  //get P_0
  std::vector<double> temporary_P;

  if (!observer_nh_.getParam("P_0", temporary_P))
    ROS_ERROR("P_0 in KF is not loaded from ros parameter server");

  for (int i = 0; i < kStateSize; i++) {
    state_covariance_(i, i) = temporary_P.at(i);
  }

  state_.setZero();

  state_.segment(0, 3) = initial_position;
  state_.segment(3, 3) = initial_velocity;
  state_.segment(6, 3) = initial_attitude;
  state_.segment(9, 3) = initial_angular_rate;
  state_.segment(12, 3) = initial_external_forces;
  state_.segment(15, 3) = initial_external_moments;
}

bool KFDisturbanceObserver::UpdateEstimator()
{
  if (initialized_ == false)
    return false;

  ROS_INFO_ONCE("KF is updated for first time.");
  static ros::Time t_previous = ros::Time::now();
  static bool do_once = true;
  double dt;

  if (do_once) {
    dt = 0.01;
    do_once = false;
  } else {
    ros::Time t0 = ros::Time::now();
    dt = (t0 - t_previous).toSec();
    t_previous = t0;
  }

  //check that dt is not so different from 0.01
  if (dt > 0.015) {
    dt = 0.015;
  }

  if (dt < 0.005) {
    dt = 0.005;
  }

  state_covariance_ = F_ * state_covariance_ * F_.transpose();
  state_covariance_.diagonal() += process_noise_covariance_;

  //predict state
  SystemDynamics(dt);

  Eigen::Matrix<double, kMeasurementSize, kMeasurementSize> tmp = H_ * state_covariance_ * H_.transpose()
      + measurement_covariance_;
  K_ = state_covariance_ * H_.transpose() * tmp.inverse();

  //Update with measurements
  state_ = predicted_state_ + K_ * (measurements_ - H_ * state_);

  //Update covariance
  state_covariance_ = (Eigen::Matrix<double, kStateSize, kStateSize>::Identity() - K_ * H_)
      * state_covariance_;

  //Limits on estimated_disturbances
  if (state_.allFinite() == false) {
    ROS_ERROR("The estimated state in KF Disturbance Observer has a non-finite element");
    return false;
  }

  Eigen::Vector3d omega = state_.segment(9, 3);
  Eigen::Vector3d external_forces = state_.segment(12, 3);
  Eigen::Vector3d external_moments = state_.segment(15, 3);

  omega = omega.cwiseMax(-omega_limit_);
  omega = omega.cwiseMin(omega_limit_);

  external_forces = external_forces.cwiseMax(-external_forces_limit_);
  external_forces = external_forces.cwiseMin(external_forces_limit_);

  external_moments = external_moments.cwiseMax(-external_moments_limit_);
  external_moments = external_moments.cwiseMin(external_moments_limit_);

  state_.segment(9, 9) << omega, external_forces, external_moments;

  if (calibrate_ == true) {
    ROS_INFO_THROTTLE(1.0, "calibrating KF...");
    forces_offset_ += external_forces;
    moments_offset_ += external_moments;
    calibration_counter_++;

    if ((ros::Time::now() - start_calibration_time_) > calibration_duration_) {
      calibrate_ = false;
      forces_offset_ = forces_offset_ / calibration_counter_;
      moments_offset_ = moments_offset_ / calibration_counter_;
      calibration_counter_ = 0;
      ROS_INFO("Calibration finished");
      std::cout << "\n\nforce_offset \n" << forces_offset_ << std::endl;
      std::cout << "\n\nmoments_offset \n" << moments_offset_ << std::endl;
    }
  }

  if (observer_state_pub_.getNumSubscribers() > 0) {
    mav_linear_mpc::ObserverStatePtr msg(new mav_linear_mpc::ObserverState);
    msg->header.stamp = ros::Time::now();
    for (int i = 0; i < 3; i++) {
      msg->position[i] = state_(i);
      msg->velocity[i] = state_(i + 3);
      msg->attitude[i] = state_(i + 6);
      msg->angular_velocity[i] = state_(i + 9);
      msg->external_forces[i] = state_(i + 12);
      msg->external_moments[i] = state_(i + 15);
    }

    observer_state_pub_.publish(msg);
  }
  return true;
}

void KFDisturbanceObserver::SystemDynamics(double dt)
{
  Eigen::Vector3d old_position = state_.segment(0, 3);
  Eigen::Vector3d old_velocity = state_.segment(3, 3);
  Eigen::Vector3d old_attitude = state_.segment(6, 3);
  Eigen::Vector3d old_omega = state_.segment(9, 3);
  Eigen::Vector3d old_external_forces = state_.segment(12, 3);
  Eigen::Vector3d old_external_moments = state_.segment(15, 3);

  const Eigen::Vector3d thrust(0.0, 0.0, this->roll_pitch_yaw_thrust_cmd_(3));

  const Eigen::Vector3d acceleration = rotation_matrix_ * thrust + Eigen::Vector3d(0, 0, -gravity_)
      + this->drag_coefficients_matrix_ * old_velocity + old_external_forces;

  const Eigen::Vector3d new_velocity = old_velocity + acceleration * dt;

  const Eigen::Vector3d new_position = old_position + old_velocity * dt + 0.5 * acceleration * dt * dt;

  Eigen::Vector3d angular_acceleration;
  angular_acceleration(0) = -2.0 * roll_damping_ * roll_omega_ * old_omega(0)
      - roll_omega_ * roll_omega_ * old_attitude(0)
      + roll_gain_ * roll_omega_ * roll_omega_ * roll_pitch_yaw_thrust_cmd_(0)
      + old_external_moments(0);

  angular_acceleration(1) = -2.0 * pitch_damping_ * pitch_omega_ * old_omega(1)
      - pitch_omega_ * pitch_omega_ * old_attitude(1)
      + pitch_gain_ * pitch_omega_ * pitch_omega_ * roll_pitch_yaw_thrust_cmd_(1)
      + old_external_moments(1);

  angular_acceleration(2) = -2.0 * yaw_damping_ * yaw_omega_ * old_omega(2)
      - yaw_omega_ * yaw_omega_ * old_attitude(2)
      + yaw_gain_ * yaw_omega_ * yaw_omega_ * roll_pitch_yaw_thrust_cmd_(2)
      + old_external_moments(2);

  const Eigen::Vector3d new_omega = old_omega + angular_acceleration * dt;

  const Eigen::Vector3d new_attitude = old_attitude + old_omega * dt
      + 0.5 * angular_acceleration * dt * dt;

  const Eigen::Vector3d new_external_forces = old_external_forces;
  const Eigen::Vector3d new_external_moments = old_external_moments;

  //Eigen::Vector3d new_external_forces = exp(-0.01/2.0 )*old_external_forces; //make external forces decay
  //Eigen::Vector3d new_external_moments = exp(-0.01/2.0 )*old_external_moments; //make external moments decay

  //Update the state vector

  predicted_state_.segment(0, 3) = new_position;
  predicted_state_.segment(3, 3) = new_velocity;
  predicted_state_.segment(6, 3) = new_attitude;
  predicted_state_.segment(9, 3) = new_omega;
  predicted_state_.segment(12, 3) = new_external_forces;
  predicted_state_.segment(15, 3) = new_external_moments;

}

void KFDisturbanceObserver::GetEstimatedState(Eigen::VectorXd* estimated_state) const
{
  assert(estimated_state);
  assert(initialized_);

  estimated_state->resize(kStateSize);
  *estimated_state = this->state_;
}

KFDisturbanceObserver::~KFDisturbanceObserver()
{
}

}
