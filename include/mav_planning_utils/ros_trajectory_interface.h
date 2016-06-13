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

#ifndef ROS_INTERFACE_H_
#define ROS_INTERFACE_H_

#include <fstream>
#include <iostream>
#include <stdio.h>

#include <mav_msgs/conversions.h>
#include <mav_viz/eigen_visualization.h>
#include <mav_viz/hexacopter_marker.h>
#include <planning_msgs/PolynomialTrajectory4D.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>

#include <mav_planning_utils/polynomial_trajectory.h>
#include <mav_planning_utils/trajectory_base.h>
#include <mav_planning_utils/trajectory_sampling.h>
#include <mav_planning_utils/trajectory_types.h>

namespace mav_planning_utils {

/**
 * \brief Draws a trajectory with visualization of velocity, and of attitude and
 * acceleration based on the
 *        flat model of a MAV.
 *
 * The headers of the markers are set to the following, i.e.:
 * - stamp: ros::Time::now()
 * - seq: not set
 * - frame_id: world
 * - marker.action: Marker::ADD
 * - marker.lifetime: 0 (i.e. infinite)
 * - marker.id: 0 ... n_markers-1
 *
 * To re-write these defaults setMarkerProperties() can be used.
 *
 * \param[in] flat_states Vector with the flat state representation of the MAV.
 *            Do not use std::vector<EigenTrajectoryPoint> (Eigen alignment
 * issues)!
 * \param[in] additional_marker mav_viz::MarkerGroup, containing additional
 * markers that should be drawn every
 *            distance. E.g. the hex-rotor markers.
 * \param[in] distance Distance between the rendering of the axes for attitude
 * and additional markers.
 * \param[out] marker_array Array with markers for trajectory visualization.
 */
bool drawMavTrajectory(
    const mav_msgs::EigenTrajectoryPoint::Vector& flat_states,
    const mav_viz::MarkerGroup& additional_marker, double distance,
    visualization_msgs::MarkerArray* marker_array);
/**
 * \brief Draws a trajectory with visualization of velocity, and of attitude and
 * acceleration based on the
 *        flat model of a MAV.
 *
 * Like above, but without additional markers and a default distance of 1m.
 */
bool drawMavTrajectory(
    const mav_msgs::EigenTrajectoryPoint::Vector& flat_states,
    visualization_msgs::MarkerArray* marker_array);

/**
 * \brief Draws a trajectory with visualization of velocity, and of attitude and
 * acceleration based on the
 *        flat model of a MAV.
 *
 * Takes trajectories for position and yaw as input, instead of flat states.
 * \sa bool drawMavTrajectory(const EigenTrajectoryPoint::Vector& flat_states,
 * const mav_viz::MarkerGroup&
 *          additional_marker, double distance, visualization_msgs::MarkerArray*
 * marker_array)
 */
bool drawMavTrajectory(const TrajectoryBase& position_trajectory,
                       const TrajectoryBase& yaw_trajectory,
                       const mav_viz::MarkerGroup& additional_marker,
                       double distance,
                       visualization_msgs::MarkerArray* marker_array);

/**
 * \brief Draws a trajectory with visualization of velocity, and of attitude and
 * acceleration based on the
 *        flat model of a MAV.
 *
 * Like above, but without additional markers and a default distance of 1m.
 */
bool drawMavTrajectory(const TrajectoryBase& position_trajectory,
                       const TrajectoryBase& yaw_trajectory,
                       visualization_msgs::MarkerArray* marker_array);

/**
 * \brief Draws a trajectory with visualization of velocity, and of attitude and
 * acceleration based on the
 *        flat model of a MAV.
 *
 * Takes a trajectory for position, instead of flat states.
 * \sa bool drawMavTrajectory(const EigenTrajectoryPoint::Vector& flat_states,
 * const mav_viz::MarkerGroup&
 *          additional_marker, double distance, visualization_msgs::MarkerArray*
 * marker_array)
 */
bool drawMavTrajectory(const TrajectoryBase& position_trajectory,
                       const mav_viz::MarkerGroup& additional_marker,
                       double distance,
                       visualization_msgs::MarkerArray* marker_array);

/**
 * \brief Draws a trajectory with visualization of velocity, and of attitude and
 * acceleration based on the
 *        flat model of a MAV.
 *
 * Like above, but without additional markers and a default distance of 1m.
 */
bool drawMavTrajectory(const TrajectoryBase& position_trajectory,
                       visualization_msgs::MarkerArray* marker_array);

/**
 * \brief Draws straight-line connections between the vertices given in the
 * list.
 *
 * The headers of the markers are set to the following, i.e.:
 * - stamp: ros::Time::now()
 * - seq: not set
 * - frame_id: world
 * - marker.action: Marker::ADD
 * - marker.lifetime: 0 (i.e. infinite)
 * - marker.id: 0 ... n_markers-1
 *
 * To re-write these defaults setMarkerProperties() can be used.
 *
 * \param[in] vertices Vector of mav_planning_utils::Vertex. Has to be of
 * dimension 3!
 * \param[out] marker_array Array with markers describing a straight-line path.
 */
bool drawVertices(const mav_planning_utils::Vertex::Vector& vertices,
                  visualization_msgs::MarkerArray* marker_array);

/**
 * \brief Overwrites the given properties of the marker array.
 */
void setMarkerProperties(const std_msgs::Header& header, double life_time,
                         const visualization_msgs::Marker::_action_type& action,
                         visualization_msgs::MarkerArray* markers);

/**
 * \brief Periodically re-publishes marker arrays.
 */
class MarkerPublisher {
 public:
  /**
   * \brief Sets up the marker publisher with period publish_period.
   */
  MarkerPublisher(ros::Publisher& publisher, double publish_period);

  /**
   * \brief Sets up the marker publisher with period publish_period and starts
   * publishing the marker array.
   */
  MarkerPublisher(ros::Publisher& publisher, double publish_period,
                  const visualization_msgs::MarkerArray& markers);

  /**
   * \brief Updates the internal marker array to publish, and immediately
   * publishes it.
   */
  void updateMarkers(const visualization_msgs::MarkerArray& markers);

 private:
  void publishMarkersCallback(const ros::TimerEvent& event);

  ros::Publisher& publisher_;
  ros::Timer publish_timer_;
  ros::Duration publish_period_;
  visualization_msgs::MarkerArray markers_;
};

/**
 * \brief Converts a planning_msgs::PolynomialTrajectory4D message into a
 * vector of polynomial segments.
 *
 * The output arguments can be set to nullptr, if not needed.
 * \note Only implemented for N=10 or N=12, everything else will yield a linker
 * error.
 */
template <int N_>
bool polynomialTrajectoryMsgToPolynomialSegments(
    const planning_msgs::PolynomialTrajectory4D& trajectory_msg,
    typename Segment<N_>::Vector* position_segments,
    typename Segment<N_>::Vector* yaw_segments);

/**
 * \brief Converts a planning_msgs::PolynomialTrajectory4D message into a
 * trajectory base.
 *
 * \note Only implemented for N=10 or N=12, everything else will yield a linker
 * error.
 */
bool polynomialTrajectoryMsgToTrajectoryBase(
    const planning_msgs::PolynomialTrajectory4D& trajectory_msg,
    mav_planning_utils::TrajectoryBase::Ptr* trajectory_position,
    mav_planning_utils::TrajectoryBase::Ptr* trajectory_yaw);

/**
 * \brief Converts a vector of polynomial position and yaw segments into a
 * planning_msgs::PolynomialTrajectory4D message.
 *
 * \note Only implemented for N=10 or N=12, everything else will yield a linker
 * error.
 */
template <int N_>
bool polynomialSegmentToPolynomialTrajectoryMsg(
    const typename Segment<N_>::Vector& position_segments,
    const typename Segment<N_>::Vector& yaw_segments,
    planning_msgs::PolynomialTrajectory4D* trajectory_msg);

/**
 * \brief Tries to converts a trajectory of position and yaw into a
 * planning_msgs::PolynomialTrajectory4D message.
 *
 * \note Only implemented for N=10 or N=12, everything else will yield a linker
 * error.
 */
bool trajectoryToPolynomialTrajectoryMsg(
    mav_planning_utils::TrajectoryBase::Ptr trajectory_position,
    mav_planning_utils::TrajectoryBase::Ptr trajectory_yaw,
    planning_msgs::PolynomialTrajectory4D* trajectory_msg);

/**
 * \brief Converts a vector of polynomial position segments into a
 * planning_msgs::PolynomialTrajectory4D message.
 *        Yaw coefficients will be set to zero.
 *
 * \note Only implemented for N=10 or N=12, everything else will yield a linker
 * error.
 */
template <int N_>
bool polynomialSegmentToPolynomialTrajectoryMsg(
    const typename Segment<N_>::Vector& position_segments,
    planning_msgs::PolynomialTrajectory4D* trajectory_msg);

/**
 * \brief Converts a trajectory base into a
 *  planning_msgs::PolynomialTrajectory4D message.
 *        Yaw coefficients will be set to zero.
 *
 * \note Only implemented for N=10 or N=12, everything else will yield a linker
 * error.
 */
bool polynomialSegmentToPolynomialTrajectoryMsg(
    const mav_planning_utils::TrajectoryBase::Ptr& trajectory,
    planning_msgs::PolynomialTrajectory4D* trajectory_msg);

template <int N_>
bool polynomialTrajectoryToFile(
    mav_planning_utils::TrajectoryBase::Ptr trajectory_position,
    mav_planning_utils::TrajectoryBase::Ptr trajectory_yaw,
    const std::string& filename);

template <int N_>
bool polynomialTrajectoryFromFile(
    const std::string& filename,
    mav_planning_utils::TrajectoryBase::Ptr* trajectory_position,
    mav_planning_utils::TrajectoryBase::Ptr* trajectory_yaw);

}  // end namespace mav_planning_utils

#endif
