/*

 Copyright (c) 2014, Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#include <limits>
#include <random>
#include <iostream>

#include <eigen-checks/gtest.h>
#include <eigen-checks/entrypoint.h>

#include <mav_planning_utils/polynomial_optimization.h>
#include <mav_planning_utils/polynomial_optimization_nonlinear.h>
#include <mav_planning_utils/polynomial_trajectory.h>
#include <mav_planning_utils/timing.h>

using namespace mav_planning_utils;

const int N = 10;
const int max_derivative = derivative_order::SNAP;
const size_t derivative_to_optimize = derivative_order::SNAP;

Eigen::IOFormat matlab_format(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[",
                              "]");

TEST(MavPlanningUtils, TrajectorySampling) {
  Eigen::Vector3d pos_min, pos_max;
  pos_min << -10.0, -20.0, -10.0;
  pos_max << 10.0, 20.0, 10.0;
  Vertex::Vector vertices;
  vertices = createRandomVertices(max_derivative, 10, pos_min, pos_max, 12345);

  std::vector<double> segment_times = {7.5, 5.06, 3, 4, 7, 8, 2, 7, 3, 5};
  double total_time = 0;
  for (double t : segment_times) total_time += t;

  PolynomialOptimization<N> opt(3);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

  opt.solveLinear();

  Segment<N>::Vector segments;
  opt.getSegments(&segments);

  PolynomialTrajectory<N> trajectory(3, segments);

  std::vector<Eigen::VectorXd> samples;
  std::vector<double> sampling_times;

  double dt = 0.1;
  int expected_number_of_samples = static_cast<int>(total_time / dt) + 1;

  trajectory.evaluateRange(0, total_time, dt, derivative_order::POSITION,
                           &samples, &sampling_times);

  EXPECT_EQ(expected_number_of_samples, samples.size());
  for (size_t i = 1; i < samples.size(); ++i) {
    EXPECT_NEAR(dt, sampling_times[i] - sampling_times[i - 1], 2.0e-9);
    double distance = (samples[i] - samples[i - 1]).norm();
    const double guessed_v_max = 10;  // just to make sure it's not 1eHuge ...
    const double expected_max_distance = guessed_v_max * dt;
    EXPECT_LT(distance, expected_max_distance);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();
  timing::Timing::Print(std::cout);
  //  createTestPolynomials();

  return result;
}
