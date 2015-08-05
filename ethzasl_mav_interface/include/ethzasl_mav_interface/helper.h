/*

Copyright (c) 2011, Markus Achtelik, ASL, ETH Zurich, Switzerland
You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef ETHZASL_MAV_INTERFACE_HELPER_H_
#define ETHZASL_MAV_INTERFACE_HELPER_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>

#include <aci/aci.h>
#include <aci/variable.h>

namespace ethzasl_mav_interface {

namespace helper {

/// checks if val exceeds [min ... max] and returns a clamped value if necessary
template<typename T>
inline T clamp(const T & min, const T & max, const T & val) {
  if (val >= max)
    return max;
  else if (val <= min)
    return min;
  else
    return val;
}

/// checks if val exceeds [min ... max] and returns a clamped value if necessary.
template<typename T>
inline T clamp(const T & min, const T & max, const T & val, bool * clamped) {
  if (val >= max) {
    *clamped = true;
    return max;
  }
  else if (val <= min) {
    *clamped = true;
    return min;
  }
  else {
    *clamped = false;
    return val;
  }
}

/// converts the yaw angle and range. AscTec uses 0...360° * 1000, we -pi...+pi
inline int yaw2asctec(const double & yaw) {
  return ((yaw < 0 ? yaw + 2 * M_PI : yaw) * 180.0 / M_PI) * 1000.0;
}

/**
 * \brief Converts RPY angles to a quaternion.
 * RPY rotates about the fixed axes in the order x-y-z,
 * which is the same as euler angles in the order z-y'-x''.
 *
 * \tparam T floating point type, i.e. double or float
 */
template<typename T>
Eigen::Quaternion<T> rpyToQuaternion(T roll, T pitch, T yaw) {
  const Eigen::AngleAxis<T> axis_roll(roll, Eigen::Matrix<T, 3, 1>::UnitX());
  const Eigen::AngleAxis<T> axis_pitch(pitch, Eigen::Matrix<T, 3, 1>::UnitY());
  const Eigen::AngleAxis<T> axis_yaw(yaw, Eigen::Matrix<T, 3, 1>::UnitZ());

  return axis_yaw * axis_pitch * axis_roll;
}

/**
 * \brief Extracts the yaw part from a quaternion, using RPY / euler (z-y'-z'') angles.
 * RPY rotates about the fixed axes in the order x-y-z,
 * which is the same as euler angles in the order z-y'-x''.
 */
template<class T>
T yawFromQuaternion(const Eigen::Quaternion<T> & q) {
  return atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

/// Rotation quaternion 180 deg around x.
const Eigen::Quaterniond q_x_pi(0.0, -1.0, 0.0, 0.0);

/// Rotation quaternion 90 deg around z.
const Eigen::Quaterniond q_z_half_pi(cos(M_PI * 0.25), 0, 0, sin(M_PI * 0.25));

/**
 * \brief Transform attitude quaternion from asctec (NED) to ros (ENU).
 */
inline Eigen::Quaterniond asctecQuaternionToRos(const Eigen::Quaterniond& q_asctec){
  // Asctec's attitude is unit quaternion, when their x points north.
  // Rotate their system -90 deg around our z axis.
  Eigen::Quaterniond q_ros = q_z_half_pi.inverse() * q_asctec;

  q_ros.vec() = helper::q_x_pi * q_ros.vec(); // faster than q_x_pi * q_ros * q_x_pi.inverse()
  return q_ros;
}

/**
 * \brief Transform attitude quaternion from ros (ENU) to asctec (NED) .
 */
inline Eigen::Quaterniond rosQuaternionToAsctec(const Eigen::Quaterniond& q_ros) {
  Eigen::Quaterniond q_asctec;
  // flip upside down first
  q_asctec.w() = q_ros.w();
  q_asctec.vec() = helper::q_x_pi * q_ros.vec();
  // then rotate 90 deg back
  return q_z_half_pi * q_asctec;
}

/**
 * \brief Converts ros local geographic representation to the representation used by AscTec.
 * In ros local geographic coordinates are expressed ENU (x east, y north, z up).
 * Yaw is zero when pointing east, and a positive rotation is towards north (CCW seen from top).
 * http://www.ros.org/reps/rep-0103.html#axis-orientation
 *
 * AscTec uses ENU for position (somewhat standard for local GPS coordinates).
 * Attitude is represented w.r.t. a North East Down system, i.e. yaw-turning towards east
 * (clockwise, seen form top) is positive.
 */
inline void rosGeographicPoseToAsctec(const Eigen::Vector3d& ros_position,
                                      const Eigen::Quaterniond& ros_orientation,
                                      Eigen::Vector3d* asctec_position,
                                      Eigen::Quaterniond* asctec_orientation) {

  if(asctec_position != nullptr){
    *asctec_position = ros_position;
  }

  if(asctec_orientation != nullptr){
    *asctec_orientation = rosQuaternionToAsctec(ros_orientation);
  }
}

/**
 * \brief Converts AscTec local geographic representation to the representation used by ros.
 * \sa rosGeographicPoseToAsctec
 */
inline void asctecGeographicPoseToRos(const Eigen::Vector3d& asctec_position,
                                      const Eigen::Quaterniond& asctec_orientation,
                                      Eigen::Vector3d* ros_position,
                                      Eigen::Quaterniond* ros_orientation) {
  if(ros_position != nullptr){
    *ros_position = asctec_position;
  }

  if(ros_orientation != nullptr){
    *ros_orientation = asctecQuaternionToRos(asctec_orientation);
  }
}

template<class Derived> geometry_msgs::Vector3 eigenVector3dToGeometryVector3(
    const Eigen::MatrixBase<Derived>& eigen) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  geometry_msgs::Vector3 vec;
  vec.x = eigen.x();
  vec.y = eigen.y();
  vec.z = eigen.z();
  return vec;
}

/// Fills the diagonal elements of a fixed 9-element array with variance, and sets all other elements to 0.
template<template<typename, size_t> class Array, typename T, size_t size>
void setDiagonal3dCovariance(double variance, Array<T, size>* covariance_matrix) {
  static_assert(size == 9, "size of covariance matrix has to be 9 (3x3)");
  covariance_matrix->fill(0.0);
  (*covariance_matrix)[0] = variance;
  (*covariance_matrix)[4] = variance;
  (*covariance_matrix)[8] = variance;
}

/// Fills the diagonal elements of a fixed 9-element array with xx, yy, zz, and sets all other elements to 0.
template<template<typename, size_t> class Array, typename T, size_t size>
void setDiagonal3dCovariance(double xx, double yy, double zz, Array<T, size>* covariance_matrix) {
  static_assert(size == 9, "size of covariance matrix has to be 9 (3x3)");
  covariance_matrix->fill(0.0);
  (*covariance_matrix)[0] = xx;
  (*covariance_matrix)[4] = yy;
  (*covariance_matrix)[8] = zz;
}

/// converts a rate in Hz to an integer period in ms.
inline uint16_t rateToPeriodMs(double rate) {
  if (rate > 0)
    return static_cast<uint16_t>(1000.0 / rate);
  else
    return 0;
}

namespace trinity{

inline double convertTime(int64_t trinity_time){
  return static_cast<double>(trinity_time) * 1.0e-6;
}

} // end namespace trinity

namespace autopilot{

/// conversion from AscTec acceleration values to m/s^2
static constexpr double kAccelerationToSi = 9.81e-4;

/// conversion from AscTec turn rates to rad/s
static constexpr double kAngularVelocityToSi = 0.0154 * M_PI / 180.0;

/// conversion from AscTec attitude angles to rad
static constexpr double kAttitudeToSi = 0.001 * M_PI / 180.0;

/// converts AscTec acceleration values to m/s^2
template<typename T>
inline double accToSI(const T & val) {
  return static_cast<double>(val) * kAccelerationToSi;
}

/// converts AscTec turn rates to rad/s
template<typename T>
inline double omegaToSI(const T & val) {
  return static_cast<double>(val) * kAngularVelocityToSi;
}

/// converts AscTec acceleration values to rad
template<typename T>
inline double attitudeToSI(const T & val) {
  return static_cast<double>(val) * kAttitudeToSi;
}

inline ros::Time convertFcuTime(const aci::Variable<aci::VariableInt64> & time) {
//  return ros::Time(static_cast<double>(time->value()) * 1e-9);
  return ros::Time().fromNSec(static_cast<uint64_t>(time.value()));
}

}  // end of namespace autopilot

}  // end of namespace helper

}  // end of namespace ethzasl_mav_interface



#endif /* HELPER_H_ */
