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

#ifndef TRAJECTORY_TYPES_H_
#define TRAJECTORY_TYPES_H_

#include <chrono>
#include <map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <glog/logging.h>

#include <mav_planning_utils/motion_defines.h>
#include <mav_planning_utils/polynomial.h>

namespace mav_planning_utils {

/**
 * \brief Vertex describes the properties of a support point of a path.
 *
 * A vertex has a set of constraints, the derivative of position a value, that
 * have to be matched
 * during optimization procedures. In case of a multi-dimensional vertex (mostly
 * 3D), the constraint
 * for a derivative exists in all dimensions, but can have different values in
 * each dimension.
 *
 * \code
 *     X------------X---------------X
 *   vertex             segment
 * \endcode
 */
class Vertex {
 public:
  typedef std::vector<Vertex> Vector;
  typedef Eigen::VectorXd ConstraintValue;
  typedef std::pair<int, ConstraintValue> Constraint;
  typedef std::map<int, ConstraintValue> Constraints;

 private:
  size_t dimension_;
  Constraints constraints_;

 public:
  /// constructs an empty vertex and sets time_to_next and
  /// derivative_to_optimize to zero
  Vertex(size_t dimension) : dimension_(dimension) {}

  size_t getDimension() const { return dimension_; }

  /**
   * \brief Adds a constraint for the specified derivative order with the given
   * value.
   * If this is a multi-dimensional vertex, all dimensions are set to the same
   * value.
   */
  void addConstraint(int derivative_order, double value) {
    constraints_[derivative_order] =
        ConstraintValue::Constant(dimension_, value);
  }

  /// adds a constraint for the derivative specified in type with the given
  /// values in c
  /**
   * the constraints for the specified derivative get set to the values in the
   * vector c. the dimension has to match the
   * dimension of the vertex
   */
  template <class Derived>
  void addConstraint(int type, const Eigen::MatrixBase<Derived>& c);

  /**
   * \brief Sets a constraint for position and sets all derivatives up to
   * (including) up_to_derivative to zero.
   * Convenience method for beginning / end vertices.
   */
  template <class Derived>
  void makeStartOrEnd(const Eigen::MatrixBase<Derived>& c,
                      int up_to_derivative);

  void makeStartOrEnd(double value, int up_to_derivative) {
    makeStartOrEnd(Eigen::VectorXd::Constant(dimension_, value),
                   up_to_derivative);
  }

  /// Returns whether the vertex has a constraint for the specified derivative
  /// order.
  bool hasConstraint(int derivative_order) const;

  /**
   * \brief Passes the value of the constraint for derivative order to *value,
   * and returns whether the
   *        constraint is set.
   */
  template <class Derived>
  bool getConstraint(int derivative_order,
                     Eigen::MatrixBase<Derived>* value) const;

  /// Returns a const iterator to the first constraint.
  typename Constraints::const_iterator cBegin() const {
    return constraints_.begin();
  }

  /// Returns a const iterator to the end of the constraints, i.e. one after the
  /// last element.
  typename Constraints::const_iterator cEnd() const {
    return constraints_.end();
  }

  /// Returns the number of constraints.
  size_t getNumberOfConstraints() const { return constraints_.size(); }

  /// Checks if both lhs and rhs are equal up to tol in case of double values.
  bool isEqualTol(const Vertex& rhs, double tol) const;
};

std::ostream& operator<<(std::ostream& stream, const Vertex& v);

std::ostream& operator<<(std::ostream& stream,
                         const std::vector<Vertex>& vertices);

/**
 * \brief Makes a rough estimate based on v_max and a_max about the time
 * required to get from one vertex to the next.
 *
 * t_est = 2 * distance/v_max * (1 + magic_fabian_constant * v_max/a_max * exp(-
 * distance/v_max * 2);
 * magic_fabian_constant was determined to 6.5 in a student project ...
 */
std::vector<double> estimateSegmentTimes(const Vertex::Vector& vertices,
                                         double v_max, double a_max,
                                         double magic_fabian_constant = 6.5);

/**
 * \brief Creates random vertices for position within minimum_position and
 * maximum_position.
 *
 * Vertices at the beginning and end have only fixed constraints with their
 * derivative set to zero, while
 * all vertices in between have position as fixed constraint and the derivatives
 * are left free.
 *
 * \param[in] maximum_derivative The maximum derivative to be set to zero for
 * beginning and end.
 * \param[in] s_segments Number of segments of the resulting trajectory. Number
 * of vertices is n_segments + 1.
 * \param[in] minimum_position Minimum position of the space to sample.
 * \param[in] maximum_position Maximum position of the space to sample.
 * \param[in] seed Initial seed for random number generation.
 * \return Vector containing n_segment + 1 vertices.
 */
template <class Derived1, class Derived2>
Vertex::Vector createRandomVertices(
    int maximum_derivative, size_t n_segments,
    const Eigen::MatrixBase<Derived1>& minimum_position,
    const Eigen::MatrixBase<Derived2>& maximum_position, size_t seed = 0);

/**
 * \brief Conveninence function to create 1D vertices.
 *
 * \sa createRandomVertices
 */
inline Vertex::Vector createRandomVertices1D(int maximum_derivative,
                                             size_t n_segments,
                                             double minimum_position,
                                             double maximum_position,
                                             size_t seed = 0);

/**
 * \brief Class holding the properties of parametric segment of a path.
 *        Time of the segment and one polynomial for each dimension.
 *
 * \code
 *     X------------X---------------X
 *   vertex             segment
 * \endcode
 */
template <int N_>
class Segment {
 public:
  enum { N = N_ };
  typedef std::vector<Segment<N>> Vector;

  Segment(size_t dimension) : time_(0), dimension_(dimension) {
    polynomials_.resize(dimension_);
  }

  double getTime() const { return time_; }
  uint64_t getTimeNSec() const { return static_cast<uint64_t>(1.0e9 * time_); }

  size_t getDimension() const { return dimension_; }

  void setTime(double _time) { time_ = _time; }

  void setTimeNSec(uint64_t time_ns) { time_ = time_ns * 1.0e-9; }

  Polynomial<N>& operator[](size_t idx);

  const Polynomial<N>& operator[](size_t idx) const;

  const std::vector<Polynomial<N>, Eigen::aligned_allocator<Polynomial<N>>>&
  getPolynomialsRef() const {
    return polynomials_;
  }

  Eigen::VectorXd evaluate(
      double t, int derivative_order = derivative_order::POSITION) const;

 protected:
  std::vector<Polynomial<N>, Eigen::aligned_allocator<Polynomial<N>>>
      polynomials_;
  double time_;

 private:
  size_t dimension_;
};

/**
 * \brief Prints the properties of the segment.
 *
 * Polynomial coefficients are printed with increasing powers, i.e. c_0 + c_1*t
 * ... c_{N-1} * t^{N-1}
 */
template <int N>
void printSegment(std::ostream& stream, const Segment<N>& s, int derivative);

template <int N>
std::ostream& operator<<(std::ostream& stream, const Segment<N>& s);

template <int N>
std::ostream& operator<<(std::ostream& stream,
                         const std::vector<Segment<N>>& segments);

/***
 * \brief Container holding the properties (time, value, segment where it
 * occurred) of an extremum.
 */
class Extremum {
 public:
  Extremum() : time(0), value(0), segment_idx(0) {}

  Extremum(double _time, double _value, int _segment_idx)
      : time(_time), value(_value), segment_idx(_segment_idx) {}

  bool operator<(const Extremum& rhs) const { return value < rhs.value; }

  double
      time;  ///< Time where the extremum occurs, relative to the segment start.
  double value;     ///< Value of the extremum at time.
  int segment_idx;  ///< Index of the segment, where the extremum occurs.
};

std::ostream& operator<<(std::ostream& stream, const Extremum& e);

}  // end namespace mav_planning_utils

#include <mav_planning_utils/implementation/trajectory_types_impl.h>

#endif
