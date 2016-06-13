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

#include <chrono>
#include <map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <mav_planning_utils/trajectory_types.h>

namespace mav_planning_utils {

bool Vertex::hasConstraint(int derivative_order) const {
  typename Constraints::const_iterator it = constraints_.find(derivative_order);
  return it != constraints_.end();
}

bool Vertex::isEqualTol(const Vertex& rhs, double tol) const {
  if (constraints_.size() != rhs.constraints_.size()) return false;
  // loop through lhs constraint map
  for (typename Constraints::const_iterator it = cBegin(); it != cEnd(); ++it) {
    // look for matching key
    typename Constraints::const_iterator rhs_it =
        rhs.constraints_.find(it->first);
    if (rhs_it == rhs.constraints_.end()) return false;
    // check value
    if (!((it->second - rhs_it->second).isZero(tol))) return false;
  }
  return true;
}

std::ostream& operator<<(std::ostream& stream, const Vertex& v) {
  stream << "constraints: " << std::endl;
  Eigen::IOFormat format(4, 0, ", ", "\n", "[", "]");
  for (typename Vertex::Constraints::const_iterator it = v.cBegin();
       it != v.cEnd(); ++it) {
    stream << "  type: " << positionDerivativeToString(it->first);
    stream << "  value: " << it->second.transpose().format(format) << std::endl;
  }
  return stream;
}

std::ostream& operator<<(std::ostream& stream,
                         const std::vector<Vertex>& vertices) {
  for (const Vertex& v : vertices) {
    stream << v << std::endl;
  }
  return stream;
}

std::vector<double> estimateSegmentTimes(const Vertex::Vector& vertices,
                                         double v_max, double a_max,
                                         double magic_fabian_constant) {
  std::vector<double> segment_times;
  segment_times.reserve(vertices.size() - 1);
  for (size_t i = 0; i < vertices.size() - 1; ++i) {
    Eigen::VectorXd start, end;
    vertices[i].getConstraint(derivative_order::POSITION, &start);
    vertices[i + 1].getConstraint(derivative_order::POSITION, &end);
    double distance = (end - start).norm();
    double t = distance / v_max * 2 * (1.0 +
                                       magic_fabian_constant * v_max / a_max *
                                           exp(-distance / v_max * 2));
    segment_times.push_back(t);
  }
  return segment_times;
}

std::ostream& operator<<(std::ostream& stream, const Extremum& e) {
  stream << "time: " << e.time << ", value: " << e.value
         << ", segment idx: " << e.segment_idx << std::endl;
  return stream;
}

}  // end namespace mav_planning_utils
