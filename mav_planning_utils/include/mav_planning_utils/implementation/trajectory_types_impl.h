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

#ifndef TRAJECTORY_TYPES_IMPL_H_
#define TRAJECTORY_TYPES_IMPL_H_

#include <chrono>
#include <map>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <glog/logging.h>

#include <mav_planning_utils/trajectory_types.h>

namespace mav_planning_utils {

template <class Derived1, class Derived2>
Vertex::Vector createRandomVertices(int maximum_derivative, size_t n_segments,
                                    const Eigen::MatrixBase<Derived1>& pos_min,
                                    const Eigen::MatrixBase<Derived2>& pos_max,
                                    size_t seed) {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived1);
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived2);

  CHECK_GE(static_cast<int>(n_segments), 1);
  CHECK_EQ(pos_min.size(), pos_min.size());
  CHECK_GT(maximum_derivative, 0);

  Vertex::Vector vertices;
  std::mt19937 generator(seed);
  std::vector<std::uniform_real_distribution<double> > distribution;

  const size_t dimension = pos_min.size();

  distribution.resize(dimension);

  for (size_t i = 0; i < dimension; ++i) {
    distribution[i] =
        std::uniform_real_distribution<double>(pos_min[i], pos_max[i]);
  }

  const double min_distance = 0.2;
  const size_t n_vertices = n_segments + 1;

  Eigen::VectorXd last_pos(dimension);
  for (size_t i = 0; i < dimension; ++i) {
    last_pos[i] = distribution[i](generator);
  }

  vertices.reserve(n_segments + 1);
  vertices.push_back(Vertex(dimension));

  vertices.front().makeStartOrEnd(last_pos, maximum_derivative);

  for (size_t i = 1; i < n_vertices; ++i) {
    Eigen::VectorXd pos(dimension);

    while (true) {
      for (size_t d = 0; d < dimension; ++d) {
        pos[d] = distribution[d](generator);
      }
      if ((pos - last_pos).norm() > min_distance) break;
    }

    Vertex v(dimension);
    v.addConstraint(derivative_order::POSITION, pos);
    vertices.push_back(v);
    last_pos = pos;
  }
  vertices.back().makeStartOrEnd(last_pos, maximum_derivative);

  return vertices;
}

inline Vertex::Vector createRandomVertices1D(int maximum_derivative,
                                             size_t n_segments, double pos_min,
                                             double pos_max, size_t seed) {
  return createRandomVertices(maximum_derivative, n_segments,
                              Eigen::Matrix<double, 1, 1>::Constant(pos_min),
                              Eigen::Matrix<double, 1, 1>::Constant(pos_max),
                              seed);
}

template <class Derived>
void Vertex::addConstraint(int derivative_order,
                           const Eigen::MatrixBase<Derived>& c) {
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived)
  CHECK(c.rows() == static_cast<long>(dimension_));
  constraints_[derivative_order] = c;
}

template <class Derived>
void Vertex::makeStartOrEnd(const Eigen::MatrixBase<Derived>& c,
                            int up_to_derivative) {
  addConstraint(derivative_order::POSITION, c);
  for (int i = 1; i <= up_to_derivative; ++i) {
    constraints_[i] = ConstraintValue::Zero(static_cast<int>(dimension_));
  }
}

template <class Derived>
bool Vertex::getConstraint(int derivative_order,
                           Eigen::MatrixBase<Derived>* value) const {
  CHECK_NOTNULL(value);
  typename Constraints::const_iterator it = constraints_.find(derivative_order);
  if (it != constraints_.end()) {
    *value = it->second;
    return true;
  } else
    return false;
}

template <int N_>
Polynomial<Segment<N_>::N>& Segment<N_>::operator[](size_t idx) {
  CHECK_LT(idx, dimension_);
  return polynomials_[idx];
}

template <int N_>
const Polynomial<Segment<N_>::N>& Segment<N_>::operator[](size_t idx) const {
  CHECK_LT(idx, dimension_);
  return polynomials_[idx];
}

template <int N_>
Eigen::VectorXd Segment<N_>::evaluate(double t, int derivative) const {
  Eigen::VectorXd result(dimension_);
  for (size_t d = 0; d < dimension_; ++d)
    result[d] = polynomials_[d].evaluate(t, derivative);

  return result;
}

template <int N>
void printSegment(std::ostream& stream, const Segment<N>& s, int derivative) {
  CHECK(derivative >= 0 && derivative < N);
  stream << "t: " << s.getTime() << std::endl;
  stream << " coefficients for " << positionDerivativeToString(derivative)
         << ": " << std::endl;
  for (size_t i = 0; i < s.getDimension(); ++i)
    stream << s[i].getCoefficients(derivative) << std::endl;
}

template <int N>
std::ostream& operator<<(std::ostream& stream, const Segment<N>& s) {
  printSegment(stream, s, derivative_order::POSITION);
  return stream;
}

template <int N>
std::ostream& operator<<(std::ostream& stream,
                         const std::vector<Segment<N> >& segments) {
  for (const Segment<N> s : segments) stream << s << std::endl;

  return stream;
}

}  // end namespace mav_planning_utils
#endif /* PATH_TYPES_H_ */
