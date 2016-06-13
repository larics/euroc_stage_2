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

#ifndef POLYNOMIAL_TRAJECTORY_H_
#define POLYNOMIAL_TRAJECTORY_H_

#include <glog/logging.h>

#include <mav_planning_utils/trajectory_base.h>
#include <mav_planning_utils/trajectory_types.h>

namespace mav_planning_utils {

template <int N_>
class PolynomialTrajectory : public TrajectoryBase {
 public:
  static constexpr int N = N_;
  typedef typename Segment<N>::Vector Segments;

  PolynomialTrajectory(size_t dimension,
                       const typename Segment<N>::Vector& segments);

  virtual ~PolynomialTrajectory();

  /**
   * \brief Writes object debug info to stream.
   */
  virtual void print(std::ostream& stream) const;

  /**
  * \brief Returns the start time of the trajectory.
  */
  virtual double getMinTime() const;

  /**
  * \brief Returns the last valid time of the trajectory.
  */
  virtual double getMaxTime() const;

  /**
  * \brief Returns the segments.
  */
  typename Segment<N_>::Vector getSegments() const;

 protected:
  /**
   * \brief Evaluates the trajectory at time t and the specified derivative.
   *
   * \param[in] t Time of evaluation.
   * \param[in] derivative_order Order of the derivative to evaluate. Position =
   * 0, velocity = 1 ...
   */
  virtual Eigen::VectorXd evaluateImpl(double t, int derivative_order) const;

  virtual void evaluateRangeImpl(double t_start, double duration, double dt,
                                 int derivative_order,
                                 std::vector<Eigen::VectorXd>* result,
                                 std::vector<double>* sampling_times) const;

  Segments segments_;
  double max_time_;
};
}

#include <mav_planning_utils/implementation/polynomial_trajectory_impl.h>

#endif /* POLYNOMIAL_TRAJECTORY_H_ */
