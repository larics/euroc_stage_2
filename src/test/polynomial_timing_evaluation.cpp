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
//#include <mav_planning_utils/testing_predicates.h>
#include <mav_planning_utils/timing.h>

using namespace mav_planning_utils;

const int N = 10;
const int max_derivative = derivative_order::SNAP;
const size_t derivative_to_optimize = derivative_order::SNAP;

Eigen::IOFormat matlab_format(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[",
                              "]");

template <class T1, class T2>
bool checkMatrices(const Eigen::MatrixBase<T1>& m1,
                   const Eigen::MatrixBase<T2>& m2, double tol) {
  return (m1 - m2).cwiseAbs().maxCoeff() < tol;
}

template <int N>
double getMaximumMagnitude(const std::vector<Segment<N> >& segments,
                           size_t derivative, double dt = 0.01) {
  double maximum = -1e9;

  for (const Segment<N>& s : segments) {
    for (double ts = 0; ts < s.getTime(); ts += dt) {
      double current_value = s.evaluate(ts, derivative).norm();
      if (current_value > maximum) maximum = current_value;
    }
  }
  return maximum;
}

template <int N>
double computeCostNumeric(const std::vector<Segment<N> >& segments,
                          size_t derivative, double dt = 0.001) {
  double cost = 0;

  for (const Segment<N>& s : segments) {
    for (double ts = 0; ts < s.getTime(); ts += dt) {
      cost += s.evaluate(ts, derivative).squaredNorm() * dt;
    }
  }
  return cost;
}

template <int N>
void checkPath(const Vertex::Vector& vertices,
               const std::vector<Segment<N> >& segments) {
  static_assert(N % 2 == 0, "N has to be even");

  const double tol = 1e-6;
  size_t n_vertices = vertices.size();
  size_t n_segments = segments.size();
  EXPECT_EQ(n_segments, n_vertices - 1);

  for (size_t i = 0; i < n_segments; ++i) {
    const Vertex& v_begin = vertices[i];
    const Vertex& v_end = vertices[i + 1];
    const Segment<N>& segment = segments[i];

    // check if fixed constraints are met.
    for (Vertex::Constraints::const_iterator it = v_begin.cBegin();
         it != v_begin.cEnd(); ++it) {
      const int derivative = it->first;
      const Vertex::ConstraintValue desired = it->second;
      const Vertex::ConstraintValue actual = segment.evaluate(0, derivative);
      Eigen::Matrix3d m, n;
      std::stringstream segment_derivative;
      printSegment(segment_derivative, segment, derivative);
      EXPECT_TRUE(EIGEN_MATRIX_NEAR(desired, actual, tol))
          << "at vertex " << i << " and constraint "
          << positionDerivativeToString(derivative) << "\nsegment:\n" << segment
          << segment_derivative.str();
    }
    for (Vertex::Constraints::const_iterator it = v_end.cBegin();
         it != v_end.cEnd(); ++it) {
      const int derivative = it->first;
      const Vertex::ConstraintValue desired = it->second;
      const Vertex::ConstraintValue actual =
          segment.evaluate(segment.getTime(), derivative);
      std::stringstream segment_derivative;
      printSegment(segment_derivative, segment, derivative);
      EXPECT_TRUE(EIGEN_MATRIX_NEAR(desired, actual, tol))
          << "at vertex " << i + 1 << " and constraint "
          << positionDerivativeToString(derivative) << "\nsegment:\n" << segment
          << segment_derivative.str();
    }

    // Check if values at vertices are continuous.
    if (i > 0) {
      const Segment<N>& last_segment = segments[i - 1];
      for (size_t derivative = 0; derivative < N / 2; ++derivative) {
        const Vertex::ConstraintValue last_segment_value =
            last_segment.evaluate(last_segment.getTime(), derivative);
        const Vertex::ConstraintValue current_segment_value =
            segment.evaluate(0, derivative);
        std::stringstream segment_derivative;
        printSegment(segment_derivative, segment, derivative);
        EXPECT_TRUE(
            EIGEN_MATRIX_NEAR(last_segment_value, current_segment_value, tol))
            << "at vertex " << i << " and constraint "
            << positionDerivativeToString(derivative) << "\nsegment:\n"
            << segment << segment_derivative.str();
      }
    }
  }
}

Vertex::Vector createRandomVerticesPath(int dimension, size_t n_segments,
                                        double average_distance,
                                        int maximum_derivative, size_t seed) {
  CHECK_GE(static_cast<int>(n_segments), 1);

  CHECK_GT(maximum_derivative, 0);

  Vertex::Vector vertices;
  std::mt19937 generator(seed);
  std::vector<std::uniform_real_distribution<double> > distribution;
  std::uniform_real_distribution<double> random_distance(0,
                                                         2 * average_distance);

  distribution.resize(dimension);

  for (size_t i = 0; i < dimension; ++i) {
    distribution[i] = std::uniform_real_distribution<double>(-1, 1);
  }

  const double min_distance = 0.2;
  const size_t n_vertices = n_segments + 1;

  Eigen::VectorXd last_position(dimension);
  for (size_t i = 0; i < dimension; ++i) {
    last_position[i] = distribution[i](generator);
  }

  vertices.reserve(n_segments + 1);
  vertices.push_back(Vertex(dimension));

  vertices.front().makeStartOrEnd(last_position, maximum_derivative);

  double distance_accumulated = 0;

  for (size_t i = 1; i < n_vertices; ++i) {
    Eigen::VectorXd position_sample(dimension);

    while (true) {
      for (size_t d = 0; d < dimension; ++d) {
        position_sample[d] = distribution[d](generator);
      }
      if (position_sample.norm() > min_distance) break;
    }

    position_sample = position_sample.normalized() * random_distance(generator);

    distance_accumulated += position_sample.norm();

    Vertex v(dimension);
    v.addConstraint(derivative_order::POSITION,
                    position_sample + last_position);
    vertices.push_back(v);
    last_position = position_sample;
  }
  vertices.back().makeStartOrEnd(last_position, maximum_derivative);

  std::cout << "average distance: "
            << distance_accumulated / static_cast<double>(n_vertices) << " vs "
            << average_distance << std::endl;

  return vertices;
}

TEST(MavPlanningUtils, PathPlanning_TestVertexGeneration3D) {
  Vertex::Vector vertices;

  vertices = createRandomVerticesPath(3, 100, 10.0, max_derivative, 12345);

  EXPECT_EQ(vertices.front().getNumberOfConstraints(), N / 2);
  EXPECT_EQ(vertices.back().getNumberOfConstraints(), N / 2);

  for (const Vertex& v : vertices) {
    EXPECT_TRUE(v.hasConstraint(derivative_order::POSITION));
    Eigen::VectorXd c;
    v.getConstraint(derivative_order::POSITION, &c);
  }
}

bool timeEval(int n_segments, double average_distance, size_t seed) {
  Vertex::Vector vertices;
  vertices = createRandomVerticesPath(3, n_segments, average_distance,
                                      max_derivative, seed);

  const double approximate_v_max = 2.0;
  const double approximate_a_max = 2.0;
  std::vector<double> segment_times = estimateSegmentTimes(
      vertices, approximate_v_max / 4.0, approximate_a_max);

  NonlinearOptimizationParameters parameters;
  parameters.max_iterations = 10000;
  parameters.f_rel = 0.05;
  parameters.x_rel = -1;
  parameters.time_penalty = 500.0;
  parameters.initial_stepsize_rel = 0.1;
  parameters.inequality_constraint_tolerance = 0.1;
  parameters.algorithm = nlopt::LN_SBPLX;
  parameters.use_soft_constraints = true;
  parameters.soft_constraint_weight = 120;
  parameters.random_seed = 12345678;

  int ret;

  PolynomialOptimizationNonLinear<N> opt(3, parameters, false);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.addMaximumMagnitudeConstraint(derivative_order::VELOCITY,
                                    approximate_v_max);
  opt.addMaximumMagnitudeConstraint(derivative_order::ACCELERATION,
                                    approximate_a_max);

  ret = opt.optimize();

  std::cout << "nlopt2 stopped for reason: " << nlopt::returnValueToString(ret)
            << std::endl;

  Segment<N>::Vector segments;

  opt.getPolynomialOptimizationRef().getSegments(&segments);

  checkPath(vertices, segments);
  double v_max = getMaximumMagnitude(segments, derivative_order::VELOCITY, 0.1);
  double a_max =
      getMaximumMagnitude(segments, derivative_order::ACCELERATION, 0.1);
  std::cout << "v_max: " << v_max << " a_max: " << a_max << std::endl;

  const double tolerance = 0.1;

  if (v_max > approximate_v_max) {
    if (v_max - approximate_v_max > approximate_v_max * tolerance) {
      std::cout << "!!!!!!! violated v: " << v_max - approximate_v_max
                << " vs. " << approximate_v_max * tolerance << std::endl;
      return false;
    }
  }
  if (a_max > approximate_a_max) {
    if (a_max - approximate_a_max > approximate_a_max * tolerance) {
      std::cout << "!!!!!!! violated a: " << a_max - approximate_a_max
                << " vs. " << approximate_a_max * tolerance << std::endl;
      return false;
    }
  }
  return true;
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  int result = RUN_ALL_TESTS();

  double n_success = 0;
  double n_fail = 0;
  int n_tries = 1;

  int n_segments = 50;
  double average_distance = 5;

  for (int i = 0; i < n_tries; ++i) {
    if (timeEval(n_segments, average_distance, i))
      n_success += 1;
    else
      n_fail += 1;
    std::cout << "# " << i << std::endl;
  }

  std::cout << "success rate for " << n_segments << " segments with avg dist "
            << average_distance << ": "
            << n_success / (n_success + n_fail) * 100 << "%\n";

  timing::Timing::Print(std::cout);
  //  createTestPolynomials();

  return result;
}
