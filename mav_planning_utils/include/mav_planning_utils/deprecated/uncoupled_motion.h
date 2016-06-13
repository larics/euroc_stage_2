/*
 * heli_model.h
 *
 *  Created on: Jan 27, 2012
 *      Author: acmarkus
 */

#ifndef UNCOUPLED_MOTION_H_
#define UNCOUPLED_MOTION_H_

#include <mav_planning_utils/dynamic_model.hpp>
#include <mav_planning_utils/mav_state.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace mav_planning_utils {

class UncoupledInternalState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void reset();

  DynamicModel<3>::x x;
  DynamicModel<3>::x y;
  DynamicModel<3>::x z;
  DynamicModel<2>::x roll;
  DynamicModel<2>::x pitch;
  DynamicModel<2>::x yaw;
};

class UncoupledMotion {
 private:
  DynamicModel<3> mdl_x_;
  DynamicModel<3> mdl_y_;
  DynamicModel<3> mdl_z_;
  DynamicModel<2> mdl_roll_;
  DynamicModel<2> mdl_pitch_;
  DynamicModel<2> mdl_yaw_;

  UncoupledInternalState int_state_;
  MavState state_;

  double location_height_;
  double location_latitude_rad_;
  double magnitude_of_gravity_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  UncoupledMotion();
  const MavState& step(const Eigen::Matrix<double, 6, 1>& goal, double dt,
                       const Eigen::Matrix<double, 6, 1>& v_max);

  void setLocationHeight(double height) {
    location_height_ = height;
    magnitude_of_gravity_ = mav_planning_utils::MagnitudeOfGravity(
        location_height_, location_latitude_rad_);
  }
  void setLocationLatitudeRadians(double latitude_radians) {
    location_latitude_rad_ = latitude_radians;
    magnitude_of_gravity_ = mav_planning_utils::MagnitudeOfGravity(
        location_height_, location_latitude_rad_);
  }
};
};

#endif /* HELI_MODEL_H_ */
