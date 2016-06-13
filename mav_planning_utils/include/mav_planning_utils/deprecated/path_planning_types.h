/*
* Copyright (c) 2014, Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#ifndef PATH_PLANNING_TYPES_H_
#define PATH_PLANNING_TYPES_H_

#include <mav_planning_utils/polynomial.h>
#include <mav_planning_utils/motion_defines.h>
#include <vector>
#include <map>
#include <glog/logging.h>
#include <Eigen/StdVector>

namespace mav_planning_utils {

namespace path_planning {

enum { DEFAULT_N = 10 };

template <int _D = 1>
class Vertex {
 public:
  const static int D = _D;
  typedef Eigen::Matrix<double, D, 1> ConstraintValueT;
  typedef std::pair<int, ConstraintValueT> Constraint;
  typedef std::map<int, ConstraintValueT, std::less<int>,
                   Eigen::aligned_allocator<Constraint> > Constraints;
  typedef std::vector<Vertex, Eigen::aligned_allocator<Vertex> > Vector;
  double time_to_next;
  int derivative_to_optimize;

 private:
  Constraints constraints;

 public:
  /// constructs an empty vertex and sets time_to_next and
  /// derivative_to_optimize to zero
  Vertex() : time_to_next(0), derivative_to_optimize(DerivativesP::none) {}

  /// constructs an empty vertex and takes arguments for time_to_next and
  /// derivative_to_optimize
  Vertex(double _time_to_next, int _derivative_to_optimze)
      : time_to_next(_time_to_next),
        derivative_to_optimize(_derivative_to_optimze) {}

  /// constructs a vertex, takes arguments for time_to_next and
  /// derivative_to_optimize; sets constraints up to derivatives to zero
  /**
   * This is useful for beginning and end vertices
   */
  Vertex(double _time_to_next, int _derivative_to_optimze, int derivatives)
      : time_to_next(_time_to_next),
        derivative_to_optimize(_derivative_to_optimze) {
    setAllZero(derivatives);
  }

  /// adds a constraint for the derivative specified in type with the given
  /// value
  /**
   * if this is a multi-dimensional vertex, all derivatives get set to the same
   * value
   */
  void addConstraint(int type, double value) {
    constraints[type] = ConstraintValueT::Constant(value);
  }

  /// adds a constraint for the derivative specified in type with the given
  /// values in c
  /**
   * the constraints for the specified derivative get set to the values in the
   * vector c. the dimension has to match the
   * dimension of the vertex
   */
  template <class Derived>
  void addConstraint(int type, const Eigen::MatrixBase<Derived>& c) {
    constraints[type] = c;
  }

  /// sets all derivatives up to (including) up_to_type to zero; useful for
  /// beginning / end vertices
  void setAllZero(int up_to_type) {
    for (int i = 0; i <= up_to_type; ++i) {
      constraints[i] = ConstraintValueT::Zero();
    }
  }

  /// returns if the vertex has a constraint specified by type
  bool hasConstraint(int type) const {
    typename Constraints::const_iterator it = constraints.find(type);
    return it != constraints.end();
  }

  /// returns the value of the constraint specified with type. If the constraint
  /// is not set, it returns zeros --> \todo
  ConstraintValueT getConstraint(int type) const {
    typename Constraints::const_iterator it = constraints.find(type);
    if (it != constraints.end())
      return it->second;
    else
      return ConstraintValueT::Zero();  // TODO : throw exception ...
  }

  /// returns a const iterator to the first constraint
  typename Constraints::const_iterator cBegin() const {
    return constraints.begin();
  }

  /// returns a const iterator to the end of the constraints, i.e. one after the
  /// last element
  typename Constraints::const_iterator cEnd() const {
    return constraints.end();
  }

  /// Returns the number of constraints.
  size_t getNumberOfConstraints() const { return constraints.size(); }

  /// Checks if both lhs and rhs are equal up to tol in case of double values.
  bool isEqualTol(const Vertex<D>& rhs, double tol) const {
    if (std::abs(time_to_next - rhs.time_to_next) > tol) return false;
    if (derivative_to_optimize != rhs.derivative_to_optimize) return false;
    if (constraints.size() != rhs.constraints.size()) return false;
    // loop through lhs constraint map
    for (typename Constraints::const_iterator it = cBegin(); it != cEnd();
         ++it) {
      // look for matching key
      typename Constraints::const_iterator rhs_it =
          rhs.constraints.find(it->first);
      if (rhs_it == rhs.constraints.end()) return false;
      // check value
      if (!((it->second - rhs_it->second).isZero(tol))) return false;
    }
    return true;
  }
};

/**
 * \brief Struct that stores waypoints and maximum values for the derivatives of
 * position / yaw
 * (if applicable). 0 means no limit.
 */
template <int _D>
class Waypoints {
 public:
  enum { D = _D };
  typedef std::vector<Vertex<D>, Eigen::aligned_allocator<Vertex<D> > >
      WaypointVector;

  double v_max;
  double a_max;
  double j_max;
  double s_max;
  double yaw_dot_max;
  WaypointVector waypoints;

  Waypoints() : v_max(0), a_max(0), j_max(0), s_max(0), yaw_dot_max(0) {}
  Waypoints(double _v_max, double _a_max, double _j_max, double _s_max,
            double _yaw_dot_max)
      : v_max(_v_max),
        a_max(_a_max),
        j_max(_j_max),
        s_max(_s_max),
        yaw_dot_max(_yaw_dot_max) {}

  /// Checks if both lhs and rhs are equal up to tol in case of double values.
  bool isEqualTol(const Waypoints<D>& rhs, double tol) const {
    if (std::abs(v_max - rhs.v_max) > tol) return false;
    if (std::abs(a_max - rhs.a_max) > tol) return false;
    if (std::abs(j_max - rhs.j_max) > tol) return false;
    if (std::abs(s_max - rhs.s_max) > tol) return false;
    if (std::abs(yaw_dot_max - rhs.yaw_dot_max) > tol) return false;

    if (waypoints.size() != rhs.waypoints.size()) return false;

    for (size_t i = 0; i < waypoints.size(); ++i) {
      if (!waypoints[i].isEqualTol(rhs.waypoints[i], tol)) return false;
    }
    return true;
  }
};

template <int D>
std::ostream& operator<<(std::ostream& stream, const Vertex<D>& v) {
  stream << "time_to_next: " << v.time_to_next << std::endl;
  stream << "derivative_to_optimize: ";
  stream << mav_planning_utils::DerivativesP::toString(v.derivative_to_optimize)
         << std::endl;
  stream << "constraints: " << std::endl;

  Eigen::IOFormat format(4, 0, ", ", "\n", "[", "]");
  for (typename Vertex<D>::Constraints::const_iterator it = v.cBegin();
       it != v.cEnd(); ++it) {
    stream << "  type: "
           << mav_planning_utils::DerivativesP::toString(it->first);
    stream << "  value: " << it->second.transpose().format(format) << std::endl;
  }
  return stream;
}

template <int D>
std::ostream& operator<<(std::ostream& stream,
                         const typename Vertex<D>::Vector& vertices) {
  for (const Vertex<D>& v : vertices) stream << v << std::endl;
}

template <int D>
std::ostream& operator<<(std::ostream& stream, const Waypoints<D>& w) {
  stream << "v_max: " << w.v_max << std::endl;
  stream << "a_max: " << w.a_max << std::endl;
  stream << "j_max: " << w.j_max << std::endl;
  stream << "s_max: " << w.s_max << std::endl;
  stream << "yaw_dot_max: " << w.yaw_dot_max << std::endl;

  stream << "vertices: " << std::endl;

  for (typename Waypoints<D>::WaypointVector::const_iterator it =
           w.waypoints.cbegin();
       it != w.waypoints.cend(); ++it) {
    stream << "  vertex: \n" << *it << std::endl;
  }
  return stream;
}

typedef Vertex<1> Vertex1D;  ///< Single axis.
typedef Vertex<3> Vertex3D;  ///< 3D position.
typedef Vertex<4> Vertex4D;  ///< 3D posiiton and yaw.

typedef Waypoints<1> Waypoints1D;  ///< Single axis.
typedef Waypoints<3> Waypoints3D;  ///< 3D position.
typedef Waypoints<4> Waypoints4D;  ///< 3D position and yaw.

template <int _N>
class Segment {
 public:
  enum { N = _N };

  typedef Polynomial<N> Type;
  typedef std::vector<Segment<N>, Eigen::aligned_allocator<Segment<N> > >
      Vector;

  Type p;
  double t;
  Segment() : t(0) {}

  Segment(const Type& _p, double _t) : p(_p), t(_t) {}
};

template <int N>
std::ostream& operator<<(std::ostream& stream, const Segment<N>& s) {
  stream << "t: " << s.t;
  stream << " coefficients: " << s.p.getCoefficients(DerivativesP::p)
         << std::endl;

  return stream;
}

template <int N>
std::ostream& operator<<(std::ostream& stream,
                         const typename Segment<N>::Vector& segments) {
  for (const Segment<N>& s : segments) stream << s << std::endl;

  return stream;
}

template <int _N>
class Segment3D {
 public:
  enum { N = _N };

  typedef Polynomial<N> Type;
  typedef std::vector<Segment3D<N>, Eigen::aligned_allocator<Segment3D<N> > >
      Vector;

  Type x, y, z;
  double t;
  Segment3D() : t(0) {}

  Segment3D(const Type& _x, const Type& _y, const Type& _z, double _t)
      : x(_x), y(_y), z(_z), t(_t) {}
};

template <int N>
std::ostream& operator<<(std::ostream& stream, const Segment3D<N>& s) {
  stream << "t: " << s.t << std::endl;
  stream << " coefficients x: " << s.x.getCoefficients(DerivativesP::p)
         << std::endl;
  stream << " coefficients y: " << s.y.getCoefficients(DerivativesP::p)
         << std::endl;
  stream << " coefficients z: " << s.z.getCoefficients(DerivativesP::p)
         << std::endl;

  return stream;
}

template <int N>
std::ostream& operator<<(std::ostream& stream,
                         const typename Segment3D<N>::Vector& segments) {
  for (const Segment3D<N>& s : segments) stream << s << std::endl;

  return stream;
}
}
}

#endif /* PATH_PLANNING_TYPES_H_ */
