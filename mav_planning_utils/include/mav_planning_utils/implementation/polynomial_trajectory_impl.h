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

#ifndef POLYNOMIAL_TRAJECTORY_IMPL_H_
#define POLYNOMIAL_TRAJECTORY_IMPL_H_

#include <mav_planning_utils/polynomial_trajectory.h>

namespace mav_planning_utils {

template <int N>
PolynomialTrajectory<N>::PolynomialTrajectory(
    size_t dimension, const typename Segment<N>::Vector& segments)
    : TrajectoryBase(dimension), max_time_(0.0) {
  CHECK_GE(dimension, size_t(1));
  for (const Segment<N>& segment : segments) {
    CHECK_EQ(segment.getDimension(), dimension_);
    max_time_ += segment.getTime();
  }

  segments_ = segments;
}

template <int N_>
PolynomialTrajectory<N_>::~PolynomialTrajectory<N_>() {}

template <int N>
void PolynomialTrajectory<N>::print(std::ostream& stream) const {
  for (const Segment<N>& segment : segments_) stream << segment << std::endl;
}

template <int N>
double PolynomialTrajectory<N>::getMinTime() const {
  return 0.0;
}

template <int N>
double PolynomialTrajectory<N>::getMaxTime() const {
  return max_time_;
}

template <int N_>
typename Segment<N_>::Vector PolynomialTrajectory<N_>::getSegments() const {
  return segments_;
}

template <int N>
Eigen::VectorXd PolynomialTrajectory<N>::evaluateImpl(
    double t, int derivative_order) const {
  typedef std::chrono::nanoseconds TimeNs;
  typedef std::chrono::duration<double> TimeDoubleS;

  TimeNs t_ns = std::chrono::duration_cast<TimeNs>(TimeDoubleS(t));

  // look for the right segment
  TimeNs accumulated_segment_time_ns(TimeNs::zero());
  typename Segment<N>::Vector::const_iterator it;
  for (it = segments_.begin(); it != segments_.end(); ++it) {
    accumulated_segment_time_ns +=
        std::chrono::duration_cast<TimeNs>(TimeDoubleS(it->getTime()));

    // |<--t_accumulated -->|
    // x----------x---------x
    //               ^t_start
    //            |chosen it|
    // in case t_start falls on a vertex, the iterator right of the vertex is
    // chosen,
    // hence accumulated_segment_time_ns > t_start
    if (accumulated_segment_time_ns > t_ns) break;
  }
  const double accumulated_segment_time =
      std::chrono::duration_cast<TimeDoubleS>(accumulated_segment_time_ns)
          .count();
  return it->evaluate(t - accumulated_segment_time + it->getTime(),
                      derivative_order);
}

template <int N>
void PolynomialTrajectory<N>::evaluateRangeImpl(
    double t_start, double duration, double dt, int derivative_order,
    std::vector<Eigen::VectorXd>* result,
    std::vector<double>* sampling_times) const {
  const bool write_sampling_times = sampling_times != nullptr;
  typedef std::chrono::nanoseconds TimeNs;
  typedef std::chrono::duration<double> TimeDoubleS;

  TimeNs t_start_ns = std::chrono::duration_cast<TimeNs>(TimeDoubleS(t_start));
  TimeNs duration_ns =
      std::chrono::duration_cast<TimeNs>(TimeDoubleS(duration));
  TimeNs dt_ns = std::chrono::duration_cast<TimeNs>(TimeDoubleS(dt));

  // One more expected sample wont hurt here, but potential reallocation later
  // could hurt.
  const int expected_number_of_samples = duration_ns / dt_ns + 1;

  result->clear();
  result->reserve(expected_number_of_samples);

  if (write_sampling_times) {
    sampling_times->clear();
    sampling_times->reserve(expected_number_of_samples);
  }

  // look for the right segment
  TimeNs accumulated_segment_time_ns(TimeNs::zero());
  typename Segment<N>::Vector::const_iterator it;
  for (it = segments_.begin(); it != segments_.end(); ++it) {
    accumulated_segment_time_ns +=
        std::chrono::duration_cast<TimeNs>(TimeDoubleS(it->getTime()));

    // |<--t_accumulated -->|
    // x----------x---------x
    //               ^t_start
    //            |chosen it|
    // in case t_start falls on a vertex, the iterator right of the vertex is
    // chosen,
    // hence accumulated_segment_time_ns > t_start
    if (accumulated_segment_time_ns > t_start_ns) break;
  }

  TimeNs accumulated_duration_ns(TimeNs::zero());
  // The first sample should always be taken.
  bool first_sample_in_segment = false;
  TimeNs current_sampling_time_ns =
      t_start_ns - accumulated_segment_time_ns +
      std::chrono::duration_cast<TimeNs>(TimeDoubleS(it->getTime()));

  while (accumulated_duration_ns <= duration_ns) {
    const TimeNs segment_time_ns =
        std::chrono::duration_cast<TimeNs>(TimeDoubleS(it->getTime()));

    if (current_sampling_time_ns > segment_time_ns) {
      current_sampling_time_ns = TimeNs::zero();
      first_sample_in_segment = true;
      ++it;
      if (it == segments_.end()) {
        LOG(ERROR) << "segment iterator == end(), this should not happen";
        return;
      }
    }

    // Skip the first sample in new segments since this is equal to the last sample in the previous segment!
    if(!first_sample_in_segment)
    {
      const double current_sampling_time =
          std::chrono::duration_cast<TimeDoubleS>(current_sampling_time_ns)
          .count();
      result->push_back(it->evaluate(current_sampling_time, derivative_order));

      if (write_sampling_times) {
        const double accumulated_duration =
            std::chrono::duration_cast<TimeDoubleS>(accumulated_duration_ns)
            .count();
        sampling_times->push_back(accumulated_duration);
      }
    }
    else
    {
      first_sample_in_segment = false;
    }
    current_sampling_time_ns += dt_ns;
    accumulated_duration_ns += dt_ns;
  }
}

}  // end namespace mav_planning_utils

#endif /* POLYNOMIAL_TRAJECTORY_IMPL_H_ */
