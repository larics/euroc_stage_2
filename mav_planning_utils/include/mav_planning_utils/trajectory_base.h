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

#ifndef TRAJECTORY_BASE_H_
#define TRAJECTORY_BASE_H_

#include <memory>

#include <Eigen/Core>
#include <glog/logging.h>

namespace mav_planning_utils {

/**
 * \brief Defines an interface to evaluate analytic representations of
 * trajectories.
 */
class TrajectoryBase {
 public:
  typedef std::shared_ptr<TrajectoryBase> Ptr;

  TrajectoryBase(size_t dimension) : dimension_(dimension) {}

  virtual ~TrajectoryBase() {}

  /**
   * \brief Returns the dimension of the trajectory.
   */
  size_t getDimension() const { return dimension_; }

  /**
   * \brief Writes object debug info to stream.
   */
  virtual void print(std::ostream& stream) const = 0;

  /**
  * \brief Returns the start time of the trajectory.
  */
  virtual double getMinTime() const = 0;

  /**
  * \brief Returns the last valid time of the trajectory.
  */
  virtual double getMaxTime() const = 0;

  /**
   * \brief Evaluates the trajectory at time t and the specified derivative.
   *
   * \param[in] t Time of evaluation.
   * \param[in] derivative_order Order of the derivative to evaluate. Position =
   * 0, velocity = 1 ...
   */
  Eigen::VectorXd evaluate(double t, int derivative_order) const;

  /**
    * \brief Evaluates the trajectory in a specified range and derivative.
    *
    * \param[in] t_start Start-time of evaluation.
    * \param[in] duration Duration of the range to evaluate.
    * \param[in] dt Time-step for the evaluation.
    * \param[in] derivative_order Order of the derivative to evaluate. Position
   * = 0, velocity = 1 ...
    * \param[out] result Vector of Eigen::VectorXd, containing the samples at
   * times t_start, t_start + dt ...
    * \param[out} sampling_times Vector constaining the times at which the
   * samples in result were taken.
    *                            Can be set to nullptr if not needed.
    */
  void evaluateRange(double t_start, double duration, double dt,
                     int derivative_order, std::vector<Eigen::VectorXd>* result,
                     std::vector<double>* sampling_times = nullptr) const;

  /**
   * \brief Evaluates the trajectory at time t and the specified derivative.
   * Skips all safety checks.
   *
   * \sa evaluate
   */
  Eigen::VectorXd evaluateUnsafe(double t, int derivative_order) const;

  /**
   * \brief Evaluates the trajectory in a specified range and derivative. Skips
   * all safety checks.
   *
   * \sa evaluateRange
   */
  void evaluateRangeUnsafe(double t_start, double duration, double dt,
                           int derivative_order,
                           std::vector<Eigen::VectorXd>* result,
                           std::vector<double>* sampling_times = nullptr) const;

 protected:
  /**
   * \brief Implements evaluate()
   * \sa evaluate
   */
  virtual Eigen::VectorXd evaluateImpl(double t,
                                       int derivative_order) const = 0;

  /**
   * \brief Implements evaluateRange()
   * \sa evaluateRange
   */
  virtual void evaluateRangeImpl(double t_start, double duration, double dt,
                                 int derivative_order,
                                 std::vector<Eigen::VectorXd>* result,
                                 std::vector<double>* sampling_times) const = 0;

  /**
   * \brief Stores the spatial dimension of the trajectory.
   */
  size_t dimension_;

  static constexpr double kDoubleTolerance = 1.0e-9;
};

inline Eigen::VectorXd TrajectoryBase::evaluate(double t,
                                                int derivative_order) const {
  CHECK_GE(derivative_order, 0);
  if (t < getMinTime() || t > getMaxTime()) {
    LOG(WARNING) << "time out of range: t = " << t << " but should be in ["
                 << getMinTime() << " ... " << getMaxTime() << "] "
                 << std::endl;
    return Eigen::VectorXd::Zero(dimension_);
  };
  return evaluateImpl(t, derivative_order);
}

inline void TrajectoryBase::evaluateRange(
    double t_start, double duration, double dt, int derivative_order,
    std::vector<Eigen::VectorXd>* result,
    std::vector<double>* sampling_times) const {
  CHECK_NOTNULL(result);
  CHECK_GE(derivative_order, 0);
  CHECK(dt > 0.0 && dt < duration);
  if (t_start < getMinTime() || t_start + duration > getMaxTime()) {
    LOG(WARNING) << "time out of range: t_start + duration = "
                 << t_start + duration << " but should be in [" << getMinTime()
                 << " ... " << getMaxTime() << "] " << std::endl;
    result->clear();
    return;
  }
  evaluateRangeImpl(t_start, duration, dt, derivative_order, result,
                    sampling_times);
}

inline Eigen::VectorXd TrajectoryBase::evaluateUnsafe(
    double t, int derivative_order) const {
  return evaluateImpl(t, derivative_order);
}

inline void TrajectoryBase::evaluateRangeUnsafe(
    double t_start, double duration, double dt, int derivative_order,
    std::vector<Eigen::VectorXd>* result,
    std::vector<double>* sampling_times) const {
  evaluateRangeImpl(t_start, duration, dt, derivative_order, result,
                    sampling_times);
}

}  // end namespace mav_planning_utils

#endif /* TRAJECTORY_BASE_H_ */
