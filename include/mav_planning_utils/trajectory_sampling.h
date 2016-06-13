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

#ifndef TRAJECTORY_SAMPLING_H_
#define TRAJECTORY_SAMPLING_H_

#include <vector>

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_planning_utils/helper.h>
#include <mav_planning_utils/trajectory_base.h>

namespace mav_planning_utils {

bool sampleTrajectory(const TrajectoryBase& trajectory_position,
                      double sample_time,
                      mav_msgs::EigenTrajectoryPoint* state);

bool sampleTrajectory(const TrajectoryBase& trajectory_position,
                      const TrajectoryBase& trajectory_yaw, double sample_time,
                      mav_msgs::EigenTrajectoryPoint* state);

bool sampleTrajectorySegment(const TrajectoryBase& trajectory_position,
                             double min_time, double max_time,
                             double sampling_interval,
                             mav_msgs::EigenTrajectoryPointVector* states);

bool sampleTrajectorySegment(const TrajectoryBase& trajectory_position,
                             const TrajectoryBase& trajectory_yaw,
                             double min_time, double max_time,
                             double sampling_interval,
                             mav_msgs::EigenTrajectoryPointVector* states);

bool sampleWholeTrajectory(const TrajectoryBase& trajectory_position,
                           double sampling_interval,
                           mav_msgs::EigenTrajectoryPoint::Vector* states);

bool sampleWholeTrajectory(const TrajectoryBase& trajectory_position,
                           const TrajectoryBase& trajectory_yaw,
                           double sampling_interval,
                           mav_msgs::EigenTrajectoryPoint::Vector* states);

}  // end namespace mav_planning_utils

#endif /* TRAJECTORY_SAMPLING_H_ */
