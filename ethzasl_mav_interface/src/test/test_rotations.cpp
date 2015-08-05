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

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen-checks/glog.h>
#include <gtest/gtest.h>
#include <random>

#include <ethzasl_mav_interface/helper.h>

using namespace ethzasl_mav_interface;

// provided by asctec
void asctecEuler2quat(double roll, double pitch, double yaw, double* q1, double* q2, double* q3, double* q4) {
  double cos_z_2 = cos(yaw * 0.5);
  double cos_y_2 = cos(pitch * 0.5);
  double cos_x_2 = cos(roll * 0.5);

  double sin_z_2 = sin(yaw * 0.5);
  double sin_y_2 = sin(pitch * 0.5);
  double sin_x_2 = sin(roll * 0.5);

  // compute quaternion
  *q1 = cos_z_2 * cos_y_2 * cos_x_2 + sin_z_2 * sin_y_2 * sin_x_2;
  *q2 = cos_z_2 * cos_y_2 * sin_x_2 - sin_z_2 * sin_y_2 * cos_x_2;
  *q3 = cos_z_2 * sin_y_2 * cos_x_2 + sin_z_2 * cos_y_2 * sin_x_2;
  *q4 = sin_z_2 * cos_y_2 * cos_x_2 - cos_z_2 * sin_y_2 * sin_x_2;
}

// converts AscTec's attitude angles to a quaternion
void myAngle2quaternion(const double &roll, const double &pitch, const double &yaw, double *w, double *x,
                             double *y, double *z)
{
  const double half_roll = roll * 0.5;
  const double half_pitch = pitch * 0.5;
  const double half_yaw = yaw * 0.5;

  const double sin_half_roll = sin(half_roll);
  const double cos_half_roll = cos(half_roll);

  const double sin_half_pitch = sin(half_pitch);
  const double cos_half_pitch = cos(half_pitch);

  const double sin_half_yaw = sin(half_yaw);
  const double cos_half_yaw = cos(half_yaw);

  // Rz*Ry*Rx
  *w = cos_half_pitch * cos_half_roll * cos_half_yaw + sin_half_pitch * sin_half_roll * sin_half_yaw;
  *x = cos_half_pitch * cos_half_yaw * sin_half_roll - cos_half_roll * sin_half_pitch * sin_half_yaw;
  *y = cos_half_roll * cos_half_yaw * sin_half_pitch + cos_half_pitch * sin_half_roll * sin_half_yaw;
  *z = cos_half_pitch * cos_half_roll * sin_half_yaw - cos_half_yaw * sin_half_pitch * sin_half_roll;
}

TEST(ethzasl_mav_interface, RotationsEuler2Quaternion) {
  const double max_roll = 45.0 * M_PI / 180.0;
  const double max_pitch = 45.0 * M_PI / 180.0;
  const double max_yaw = 180.0 * M_PI / 180.0;
  const double increment = 1.0 * M_PI / 180.0;

  const double double_eq_tolerance = 1.0e-14;

  for (double roll = -max_roll; roll < max_roll; roll += increment) {
    for (double pitch = -max_pitch; pitch < max_pitch; pitch += increment) {
      for (double yaw = -max_roll; yaw < max_yaw; yaw += increment) {
        double q_w_ref,q_x_ref,q_y_ref,q_z_ref;
        double q1_w, q1_x, q1_y, q1_z;

        asctecEuler2quat(roll, pitch, yaw, &q_w_ref, &q_x_ref, &q_y_ref, &q_z_ref);
        myAngle2quaternion(roll, pitch, yaw, &q1_w, &q1_x, &q1_y, &q1_z);
        Eigen::Quaterniond q2 = helper::rpyToQuaternion(roll, pitch, yaw);

        EXPECT_NEAR(q_w_ref, q1_w, double_eq_tolerance);
        EXPECT_NEAR(q_x_ref, q1_x, double_eq_tolerance);
        EXPECT_NEAR(q_y_ref, q1_y, double_eq_tolerance);
        EXPECT_NEAR(q_z_ref, q1_z, double_eq_tolerance);

        EXPECT_NEAR(q_w_ref, q2.w(), double_eq_tolerance);
        EXPECT_NEAR(q_x_ref, q2.x(), double_eq_tolerance);
        EXPECT_NEAR(q_y_ref, q2.y(), double_eq_tolerance);
        EXPECT_NEAR(q_z_ref, q2.z(), double_eq_tolerance);
      }
    }
  }
}

TEST(ethzasl_mav_interface, ConversionFromAsctecAndBack) {
  const size_t seed = 7983;
  const double max_val = 10.0;
  const int n_tries = 100;

  const double double_eq_tolerance = 1.0e-14;

  std::mt19937 generator(seed);
  std::uniform_real_distribution<double> uniform_distribution(-max_val, max_val);

  for (int i = 0; i < n_tries; ++i) {
    Eigen::Vector3d position(uniform_distribution(generator), uniform_distribution(generator),
                             uniform_distribution(generator));
    Eigen::Quaterniond attitude(uniform_distribution(generator), uniform_distribution(generator),
                                uniform_distribution(generator), uniform_distribution(generator));
    attitude.normalize();

    Eigen::Quaterniond q1, q2;
    q1 = helper::rosQuaternionToAsctec(attitude);
    q1 = helper::asctecQuaternionToRos(q1);
    CHECK_EIGEN_MATRIX_NEAR(attitude.coeffs(), q1.coeffs(), double_eq_tolerance)
        << "quaternion ros-->asctec-->ros check failed";

    q1 = helper::asctecQuaternionToRos(attitude);
    q1 = helper::rosQuaternionToAsctec(q1);
    CHECK_EIGEN_MATRIX_NEAR(attitude.coeffs(), q1.coeffs(), double_eq_tolerance)
        << "quaternion asctec-->ros-->asctec check failed";

    Eigen::Vector3d p1, p2;
    helper::asctecGeographicPoseToRos(position, attitude, &p1, &q1);
    helper::rosGeographicPoseToAsctec(p1, q1, &p2, &q2);
    CHECK_EIGEN_MATRIX_NEAR(attitude.coeffs(), q2.coeffs(), double_eq_tolerance)
        << "pose asctec-->ros-->asctec check failed";
    CHECK_EIGEN_MATRIX_NEAR(position, p2, double_eq_tolerance)
        << "pose asctec-->ros-->asctec check failed";
  }
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
