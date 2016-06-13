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

#include <mav_planning_utils/trajectory_sampling.h>

using namespace mav_planning_utils::derivative_order;

namespace mav_planning_utils {

bool sampleTrajectory(const TrajectoryBase& trajectory_position,
                      double sample_time,
                      mav_msgs::EigenTrajectoryPoint* state) {
  CHECK_NOTNULL(state);
  if (sample_time < trajectory_position.getMinTime() ||
      sample_time > trajectory_position.getMaxTime()) {
    std::stringstream msg;
    msg << "Sample time should be within [" << trajectory_position.getMinTime()
        << " " << trajectory_position.getMaxTime() << "] but is "
        << sample_time;
    LOG(ERROR) << msg.str();
    return false;
  }

  if (trajectory_position.getDimension() != 3) {
    LOG(ERROR) << "Dimension has to be 3, but is "
               << trajectory_position.getDimension();
    return false;
  }

  state->position_W = trajectory_position.evaluate(sample_time, POSITION);
  state->velocity_W = trajectory_position.evaluate(sample_time, VELOCITY);
  state->acceleration_W =
      trajectory_position.evaluate(sample_time, ACCELERATION);
  state->jerk_W = trajectory_position.evaluate(sample_time, JERK);
  state->snap_W = trajectory_position.evaluate(sample_time, SNAP);

  state->setFromYaw(0.0);
  state->setFromYawRate(0.0);
  state->time_from_start_ns = static_cast<int64_t>(sample_time * 1.0e9);

  return true;
}

bool sampleTrajectory(const TrajectoryBase& trajectory_position,
                      const TrajectoryBase& trajectory_yaw, double sample_time,
                      mav_msgs::EigenTrajectoryPoint* state) {
  // Check if there is a valid yaw trajectory here.
  if (trajectory_yaw.getMaxTime() - trajectory_yaw.getMinTime() <= 0) {
    return sampleTrajectory(trajectory_position, sample_time, state);
  }

  const double kMaxTolerance = 1e-6;

  const bool t_start_match =
      std::abs(trajectory_position.getMinTime() - trajectory_yaw.getMinTime()) <
      kMaxTolerance;
  const bool t_end_match =
      std::abs(trajectory_position.getMaxTime() - trajectory_yaw.getMaxTime()) <
      kMaxTolerance;
  if (!(t_start_match && t_end_match)) {
    std::stringstream msg;
    msg << "start and / or end times of position and yaw trajectory do not "
           "match: "
        << trajectory_position.getMinTime() << "/"
        << trajectory_yaw.getMinTime() << " "
        << trajectory_position.getMaxTime() << "/"
        << trajectory_yaw.getMaxTime();

    LOG(ERROR) << msg.str();
    return false;
  }
  sampleTrajectory(trajectory_position, sample_time, state);

  state->setFromYaw(trajectory_yaw.evaluate(sample_time, ORIENTATION)[0]);
  state->setFromYawRate(
      trajectory_yaw.evaluate(sample_time, ANGULAR_VELOCITY)[0]);
  state->time_from_start_ns = static_cast<int64_t>(sample_time * 1.0e9);

  return true;
}

bool sampleTrajectorySegmentImpl(const TrajectoryBase& trajectory_position,
                                 const TrajectoryBase* trajectory_yaw,
                                 double min_time, double max_time,
                                 double sampling_interval,
                                 mav_msgs::EigenTrajectoryPointVector* states);

bool sampleTrajectorySegment(const TrajectoryBase& trajectory_position,
                             double min_time, double max_time,
                             double sampling_interval,
                             mav_msgs::EigenTrajectoryPoint::Vector* states) {
  return sampleTrajectorySegmentImpl(trajectory_position, nullptr, min_time,
                                     max_time, sampling_interval, states);
}

bool sampleTrajectorySegment(const TrajectoryBase& trajectory_position,
                             const TrajectoryBase& trajectory_yaw,
                             double min_time, double max_time,
                             double sampling_interval,
                             mav_msgs::EigenTrajectoryPointVector* states) {
  return sampleTrajectorySegmentImpl(trajectory_position, &trajectory_yaw,
                                     min_time, max_time, sampling_interval,
                                     states);
}

bool sampleTrajectorySegmentImpl(const TrajectoryBase& trajectory_position,
                                 const TrajectoryBase* trajectory_yaw,
                                 double min_time, double max_time,
                                 double sampling_interval,
                                 mav_msgs::EigenTrajectoryPointVector* states) {
  const bool use_yaw =
      (trajectory_yaw != nullptr &&
       trajectory_yaw->getMinTime() != trajectory_yaw->getMaxTime());

  const double duration = max_time - min_time;

  if (use_yaw) {
    const bool t_start_match =
        trajectory_position.getMinTime() == trajectory_yaw->getMinTime();
    const bool t_end_match =
        trajectory_position.getMaxTime() == trajectory_yaw->getMaxTime();
    if (!(t_start_match && t_end_match)) {
      std::stringstream msg;
      msg << "start and / or end times of position and yaw trajectory do not "
             "match: "
          << trajectory_position.getMinTime() << "/"
          << trajectory_yaw->getMinTime() << " "
          << trajectory_position.getMaxTime() << "/"
          << trajectory_yaw->getMaxTime();

      LOG(ERROR) << msg.str();
      return false;
    }

    if (trajectory_yaw->getDimension() != 1) {
      LOG(ERROR) << "yaw trajectory should be 1D, but is "
                 << trajectory_yaw->getDimension();
      return false;
    }
  }

  if (trajectory_position.getDimension() != 3) {
    LOG(ERROR) << "position trajectory should be 3D, but is "
               << trajectory_position.getDimension();
    return false;
  }

  std::vector<Eigen::VectorXd> position, velocity, acceleration, jerk, snap,
      yaw, yaw_rate;

  trajectory_position.evaluateRange(min_time, duration, sampling_interval,
                                    POSITION, &position);
  trajectory_position.evaluateRange(min_time, duration, sampling_interval,
                                    VELOCITY, &velocity);
  trajectory_position.evaluateRange(min_time, duration, sampling_interval,
                                    ACCELERATION, &acceleration);
  trajectory_position.evaluateRange(min_time, duration, sampling_interval, JERK,
                                    &jerk);
  trajectory_position.evaluateRange(min_time, duration, sampling_interval, SNAP,
                                    &snap);

  size_t n_samples = position.size();

  if (use_yaw) {
    trajectory_yaw->evaluateRange(min_time, duration, sampling_interval,
                                  ORIENTATION, &yaw);
    trajectory_yaw->evaluateRange(min_time, duration, sampling_interval,
                                  ANGULAR_VELOCITY, &yaw_rate);
    const size_t n_samples_yaw = yaw.size();

    if (n_samples_yaw != n_samples) {
      std::stringstream msg;
      msg << "position and yaw samples have different sizes: " << n_samples
          << "/" << n_samples_yaw << ". Using minimum";
      LOG(WARNING) << msg.str();
      n_samples = std::min(n_samples, n_samples_yaw);
    }
  }

  states->resize(n_samples);
  for (size_t i = 0; i < n_samples; ++i) {
    mav_msgs::EigenTrajectoryPoint& state = (*states)[i];

    state.position_W = position[i];
    state.velocity_W = velocity[i];
    state.acceleration_W = acceleration[i];
    state.jerk_W = jerk[i];
    state.snap_W = snap[i];
    state.time_from_start_ns =
        static_cast<int64_t>((min_time + sampling_interval * i) * 1.0e9);
    if (use_yaw) {
      state.setFromYaw(yaw[i][0]);
      state.setFromYawRate(yaw_rate[i][0]);
    } else {
      state.setFromYaw(0.0);
      state.setFromYawRate(0.0);
    }
  }

  return true;
}

bool sampleWholeTrajectoryImpl(const TrajectoryBase& trajectory_position,
                               const TrajectoryBase* trajectory_yaw,
                               double sampling_interval,
                               mav_msgs::EigenTrajectoryPointVector* states);

bool sampleWholeTrajectory(const TrajectoryBase& trajectory_position,
                           double sampling_interval,
                           mav_msgs::EigenTrajectoryPointVector* states) {
  return sampleWholeTrajectoryImpl(trajectory_position, nullptr,
                                   sampling_interval, states);
}

bool sampleWholeTrajectory(const TrajectoryBase& trajectory_position,
                           const TrajectoryBase& trajectory_yaw,
                           double sampling_interval,
                           mav_msgs::EigenTrajectoryPointVector* states) {
  return sampleWholeTrajectoryImpl(trajectory_position, &trajectory_yaw,
                                   sampling_interval, states);
}

bool sampleWholeTrajectoryImpl(const TrajectoryBase& trajectory_position,
                               const TrajectoryBase* trajectory_yaw,
                               double sampling_interval,
                               mav_msgs::EigenTrajectoryPointVector* states) {
  const double min_time = trajectory_position.getMinTime();
  const double max_time = trajectory_position.getMaxTime();

  bool success =
      sampleTrajectorySegmentImpl(trajectory_position, trajectory_yaw, min_time,
                                  max_time, sampling_interval, states);

  return success;
}
}
