/*
 * Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef HELPER_H_
#define HELPER_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <mav_planning_utils/implementation/sincos.h>
#include <mav_planning_utils/motion_defines.h>

namespace mav_planning_utils {

/// Magnitude of Earth's gravitational field at specific height [m] and latitude
/// [rad] (from wikipedia).
inline double MagnitudeOfGravity(const double height,
                                 const double latitude_radians) {
  double sin_squared_latitude = sin(latitude_radians) * sin(latitude_radians);
  double sin_squared_twice_latitude =
      sin(2 * latitude_radians) * sin(2 * latitude_radians);
  return 9.780327 * ((1 + 0.0053024 * sin_squared_latitude -
                      0.0000058 * sin_squared_twice_latitude) -
                     3.155 * 1e-7 * height);
}

template <class T>
T yawFromQuaternion(const Eigen::Quaternion<T>& q) {
  return atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
               1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

template <class T>
Eigen::Quaternion<T> quaternionFromYaw(T yaw) {
  const T yaw_2 = yaw / 2.0;
  return Eigen::Quaternion<T>(cos(yaw_2), 0, 0, sin(yaw_2));
}

template <class Derived>
Eigen::Quaternion<typename Derived::Scalar> quaternionFromSmallAngle(
    const Eigen::MatrixBase<Derived>& theta) {
  typedef typename Derived::Scalar Scalar;
  EIGEN_STATIC_ASSERT_FIXED_SIZE(Derived);
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
  const Scalar q_squared = theta.squaredNorm() / 4.0;

  if (q_squared < 1) {
    return Eigen::Quaternion<Scalar>(sqrt(1 - q_squared), theta[0] * 0.5,
                                     theta[1] * 0.5, theta[2] * 0.5);
  } else {
    const Scalar w = 1.0 / sqrt(1 + q_squared);
    const Scalar f = w * 0.5;
    return Eigen::Quaternion<Scalar>(w, theta[0] * f, theta[1] * f,
                                     theta[2] * f);
  }
}

template <int D, int K>
struct ConvolutionDimension {
  enum { length = D + K - 1 };
};

template <int DataDimension_, int KernelDimension_>
Eigen::Matrix<double,
              ConvolutionDimension<DataDimension_, KernelDimension_>::length, 1>
convolve(const Eigen::Matrix<double, DataDimension_, 1>& data,
         const Eigen::Matrix<double, KernelDimension_, 1>& kernel) {
  const int convolution_dimension =
      ConvolutionDimension<DataDimension_, KernelDimension_>::length;
  Eigen::Matrix<double, convolution_dimension, 1> convolved;
  convolved.setZero();
  Eigen::Matrix<double, KernelDimension_, 1> kernel_reverse(kernel.reverse());

  for (int output_idx = 0; output_idx < convolution_dimension; ++output_idx) {
    const int data_idx = output_idx - KernelDimension_ + 1;

    int lower_bound = std::max(0, -data_idx);
    int upper_bound = std::min(KernelDimension_, DataDimension_ - data_idx);

    for (int kernel_idx = lower_bound; kernel_idx < upper_bound; ++kernel_idx) {
      convolved[output_idx] +=
          kernel_reverse[kernel_idx] * data[data_idx + kernel_idx];
    }
  }

  return convolved;
}

template <typename T>
int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}
}
#endif /* HELPER_H_ */
