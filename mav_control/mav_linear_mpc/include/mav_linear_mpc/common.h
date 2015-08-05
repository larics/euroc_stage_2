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

#ifndef INCLUDE_MAV_LINEAR_MPC_COMMON_H_
#define INCLUDE_MAV_LINEAR_MPC_COMMON_H_

#include <nav_msgs/Odometry.h>
#include <mav_msgs/conversions.h>
#include <ros/ros.h>
#include <memory.h>
#include <deque>

namespace mav_control {

inline void quat2rpy(const Eigen::Quaternion<double> &q, Eigen::Vector3d* rpy)
{
  assert(rpy);

  *rpy << atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y())), asin(
      2.0 * (q.w() * q.y() - q.z() * q.x())), atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                                                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

}

#endif /* INCLUDE_MAV_LINEAR_MPC_COMMON_H_ */
