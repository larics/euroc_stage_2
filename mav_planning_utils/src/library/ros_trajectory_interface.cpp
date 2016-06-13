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

#include <mav_planning_utils/ros_trajectory_interface.h>
#include <mav_planning_utils/trajectory_sampling.h>
#include <planning_msgs/conversions.h>

namespace mav_planning_utils {

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

using namespace derivative_order;

// declare/define some internal functions that we don't want to expose
namespace internal {

static constexpr double kDefaultDistance = 1.0;
static constexpr double kDefaultSamplingInterval = 0.1;

void appendMarkers(const MarkerArray& markers_to_insert,
                   const std::string& marker_namespace,
                   MarkerArray* marker_array);

void appendMarkers(const MarkerArray& markers_to_insert,
                   const std::string& marker_namespace,
                   MarkerArray* marker_array) {
  marker_array->markers.reserve(marker_array->markers.size() +
                                markers_to_insert.markers.size());
  for (const auto marker : markers_to_insert.markers) {
    marker_array->markers.push_back(marker);
    if (!marker_namespace.empty()) {
      marker_array->markers.back().ns = marker_namespace;
    }
  }
}

void appendMarkers(const MarkerArray& markers_to_insert,
                   MarkerArray* marker_array) {
  appendMarkers(markers_to_insert, "", marker_array);
}

}  // end namespace internal

bool drawMavTrajectory(
    const mav_msgs::EigenTrajectoryPoint::Vector& flat_states,
    visualization_msgs::MarkerArray* marker_array) {
  mav_viz::MarkerGroup dummy_marker;
  return drawMavTrajectory(flat_states, dummy_marker,
                           internal::kDefaultDistance, marker_array);
}

bool drawMavTrajectory(
    const mav_msgs::EigenTrajectoryPoint::Vector& flat_states,
    const mav_viz::MarkerGroup& additional_marker, double distance,
    visualization_msgs::MarkerArray* marker_array) {
  CHECK_NOTNULL(marker_array);
  marker_array->markers.clear();
  const size_t n_samples = flat_states.size();

  Marker line_strip;
  line_strip.type = Marker::LINE_STRIP;
  line_strip.color = mav_viz::createColorRGBA(1, 0.5, 0, 1);
  line_strip.scale.x = 0.01;
  line_strip.ns = "path";

  double accumulated_distance = 0;
  Eigen::Vector3d last_position = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < n_samples; ++i) {
    const mav_msgs::EigenTrajectoryPoint& flat_state = flat_states[i];

    accumulated_distance += (last_position - flat_state.position_W).norm();
    if (accumulated_distance > distance) {
      accumulated_distance = 0.0;
      mav_msgs::EigenMavState mav_state;
      mav_msgs::EigenMavStateFromEigenTrajectoryPoint(flat_state, &mav_state);

      MarkerArray axes_arrows;
      mav_viz::drawAxesArrows(axes_arrows, mav_state.position_W,
                              mav_state.orientation_W_B, 0.3, 0.3);
      internal::appendMarkers(axes_arrows, "pose", marker_array);

      Marker arrow;
      mav_viz::drawArrow(
          arrow, flat_state.position_W,
          flat_state.position_W + flat_state.acceleration_W,
          mav_viz::createColorRGBA((190.0 / 255.0), (81.0 / 255.0),
                                   (80.0 / 255.0), 1),
          0.3);
      arrow.ns = positionDerivativeToString(ACCELERATION);
      marker_array->markers.push_back(arrow);

      mav_viz::drawArrow(
          arrow, flat_state.position_W,
          flat_state.position_W + flat_state.velocity_W,
          mav_viz::createColorRGBA((80.0 / 255.0), (172.0 / 255.0),
                                   (196.0 / 255.0), 1),
          0.3);
      arrow.ns = positionDerivativeToString(VELOCITY);
      marker_array->markers.push_back(arrow);

      mav_viz::MarkerGroup tmp_marker(additional_marker);
      tmp_marker.transform(mav_state.position_W, mav_state.orientation_W_B);
      tmp_marker.getMarkers(marker_array->markers, 1.0, true);
    }
    last_position = flat_state.position_W;
    line_strip.points.push_back(mav_viz::eigenToPoint(last_position));
  }
  marker_array->markers.push_back(line_strip);

  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time::now();
  setMarkerProperties(header, 0.0, Marker::ADD, marker_array);

  return true;
}

void setMarkerProperties(const std_msgs::Header& header, double life_time,
                         const Marker::_action_type& action,
                         MarkerArray* markers) {
  CHECK_NOTNULL(markers);
  int count = 0;
  for (Marker& marker : markers->markers) {
    marker.header = header;
    marker.action = action;
    marker.id = count;
    marker.lifetime = ros::Duration(life_time);
    ++count;
  }
}

bool drawMavTrajectory(const TrajectoryBase& position_trajectory,
                       const TrajectoryBase& yaw_trajectory,
                       const mav_viz::MarkerGroup& additional_marker,
                       double distance,
                       visualization_msgs::MarkerArray* marker_array) {
  CHECK_NOTNULL(marker_array);
  mav_msgs::EigenTrajectoryPoint::Vector flat_states;
  bool ret = mav_planning_utils::sampleWholeTrajectory(
      position_trajectory, yaw_trajectory, internal::kDefaultSamplingInterval,
      &flat_states);
  if (ret) {
    return drawMavTrajectory(flat_states, additional_marker, distance,
                             marker_array);
  }
  return false;
}

bool drawMavTrajectory(const TrajectoryBase& position_trajectory,
                       const TrajectoryBase& yaw_trajectory,
                       visualization_msgs::MarkerArray* marker_array) {
  CHECK_NOTNULL(marker_array);
  mav_viz::MarkerGroup dummy_marker;
  return drawMavTrajectory(position_trajectory, yaw_trajectory, dummy_marker,
                           internal::kDefaultDistance, marker_array);
}

bool drawMavTrajectory(const TrajectoryBase& position_trajectory,
                       const mav_viz::MarkerGroup& additional_marker,
                       double distance,
                       visualization_msgs::MarkerArray* marker_array) {
  CHECK_NOTNULL(marker_array);
  mav_msgs::EigenTrajectoryPoint::Vector flat_states;
  bool ret = mav_planning_utils::sampleWholeTrajectory(
      position_trajectory, internal::kDefaultSamplingInterval, &flat_states);
  if (ret) {
    return drawMavTrajectory(flat_states, additional_marker, distance,
                             marker_array);
  }
  return false;
}

bool drawMavTrajectory(const TrajectoryBase& position_trajectory,
                       visualization_msgs::MarkerArray* marker_array) {
  CHECK_NOTNULL(marker_array);
  mav_viz::MarkerGroup dummy_marker;
  return drawMavTrajectory(position_trajectory, dummy_marker,
                           internal::kDefaultDistance, marker_array);
}

bool drawVertices(const mav_planning_utils::Vertex::Vector& vertices,
                  visualization_msgs::MarkerArray* marker_array) {
  CHECK_NOTNULL(marker_array);
  marker_array->markers.resize(1);
  Marker& marker = marker_array->markers.front();

  marker.type = Marker::LINE_STRIP;
  marker.color = mav_viz::createColorRGBA(0.5, 1.0, 0, 1);
  marker.scale.x = 0.01;
  marker.ns = "straight_path";

  for (const Vertex& vertex : vertices) {
    if (vertex.getDimension() != 3) {
      ROS_ERROR("Vertex has dimension %lu but should have dimension 3.",
                vertex.getDimension());
      return false;
    }

    if (vertex.hasConstraint(POSITION)) {
      Eigen::Vector3d position = Eigen::Vector3d::Zero();
      vertex.getConstraint(POSITION, &position);
      marker.points.push_back(mav_viz::eigenToPoint(position));
    } else
      ROS_WARN("Vertex does not have a position constraint, skipping.");
  }

  std_msgs::Header header;
  header.frame_id = "world";
  header.stamp = ros::Time::now();
  setMarkerProperties(header, 0.0, Marker::ADD, marker_array);
  return true;
}

MarkerPublisher::MarkerPublisher(ros::Publisher& publisher,
                                 double publish_period)
    : publisher_(publisher), publish_period_(publish_period) {
  ros::NodeHandle nh;
  publish_timer_ = nh.createTimer(
      publish_period_, &MarkerPublisher::publishMarkersCallback, this);
  publish_timer_.stop();
}

MarkerPublisher::MarkerPublisher(ros::Publisher& publisher,
                                 double publish_period,
                                 const visualization_msgs::MarkerArray& markers)
    : publisher_(publisher), publish_period_(publish_period) {
  ros::NodeHandle nh;
  publish_timer_ = nh.createTimer(
      publish_period_, &MarkerPublisher::publishMarkersCallback, this);
  updateMarkers(markers);
}

void MarkerPublisher::updateMarkers(
    const visualization_msgs::MarkerArray& markers) {
  markers_ = markers;
  for (Marker& marker : markers_.markers) {
    marker.lifetime =
        publish_period_ *
        1.1;  // let them live 10% longer than the republish-period
  }
  if (publisher_) {
    publisher_.publish(markers_);
  }
  publish_timer_.start();
}

void MarkerPublisher::publishMarkersCallback(const ros::TimerEvent& event) {
  if (publisher_ && markers_.markers.size() > 0) {
    ros::Time time_now = ros::Time::now();
    for (visualization_msgs::Marker& marker : markers_.markers) {
      marker.header.stamp = time_now;
    }
    visualization_msgs::MarkerArrayPtr msg(
        new visualization_msgs::MarkerArray(markers_));
    publisher_.publish(msg);
  }
}

template <int N_>
bool polynomialTrajectoryMsgToPolynomialSegments(
    const planning_msgs::PolynomialTrajectory4D& trajectory_msg,
    typename Segment<N_>::Vector* position_segments,
    typename Segment<N_>::Vector* yaw_segments) {
  const bool fill_position = position_segments != nullptr;
  const bool fill_yaw = (yaw_segments != nullptr && !trajectory_msg.segments.empty());

  if (fill_position) position_segments->reserve(trajectory_msg.segments.size());

  if (fill_yaw) yaw_segments->reserve(trajectory_msg.segments.size());

  planning_msgs::EigenPolynomialTrajectory eigen_trajectory_msg;
  planning_msgs::eigenPolynomialTrajectoryFromMsg(trajectory_msg,
                                                  &eigen_trajectory_msg);

  bool success = true;
  for (const planning_msgs::EigenPolynomialSegment& waypoint :
       eigen_trajectory_msg) {
    if (fill_position) {
      if (waypoint.x.size() != N_ || waypoint.y.size() != N_ ||
          waypoint.z.size() != N_) {
        ROS_ERROR("Number of coefficients has to be %d but is %lu / %lu / %lu.",
                  N_, waypoint.x.size(), waypoint.y.size(), waypoint.z.size());
        success = false;
        break;
      }
      Segment<N_> position_segment(3);
      position_segment[0].setCoefficients(waypoint.x);
      position_segment[1].setCoefficients(waypoint.y);
      position_segment[2].setCoefficients(waypoint.z);
      position_segment.setTimeNSec(waypoint.segment_time_ns);
      position_segments->push_back(position_segment);
    }
    if (fill_yaw) {
      if (waypoint.yaw.size() != N_) {
        ROS_ERROR("Number of coefficients has to be %d but is %lu.", N_,
                  waypoint.yaw.size());
        success = false;
        break;
      }
      Segment<N_> yaw_segment(1);
      yaw_segment[0].setCoefficients(waypoint.yaw);
      yaw_segment.setTimeNSec(waypoint.segment_time_ns);
      yaw_segments->push_back(yaw_segment);
    }
  }

  if (!success) {
    if (fill_position) position_segments->clear();
    if (fill_yaw) yaw_segments->clear();
    return false;
  }

  return true;
}

template bool polynomialTrajectoryMsgToPolynomialSegments<10>(
    const planning_msgs::PolynomialTrajectory4D& trajectory_msg,
    Segment<10>::Vector* position_segments, Segment<10>::Vector* yaw_segments);

template bool polynomialTrajectoryMsgToPolynomialSegments<12>(
    const planning_msgs::PolynomialTrajectory4D& trajectory_msg,
    Segment<12>::Vector* position_segments, Segment<12>::Vector* yaw_segments);

bool polynomialTrajectoryMsgToTrajectoryBase(
    const planning_msgs::PolynomialTrajectory4D& msg,
    mav_planning_utils::TrajectoryBase::Ptr* trajectory_position,
    mav_planning_utils::TrajectoryBase::Ptr* trajectory_yaw) {
  bool success = false;

  int num_coeffs = msg.segments[0].num_coeffs;
  // type type type type type = type type
  if (num_coeffs == 12) {
    mav_planning_utils::Segment<12>::Vector position, yaw;
    success =
        mav_planning_utils::polynomialTrajectoryMsgToPolynomialSegments<12>(
            msg, &position, &yaw);
    if (success) {
      trajectory_position->reset(
          new mav_planning_utils::PolynomialTrajectory<12>(3, position));
      trajectory_yaw->reset(
          new mav_planning_utils::PolynomialTrajectory<12>(1, yaw));
    }
  } else if (num_coeffs == 10) {
    mav_planning_utils::Segment<10>::Vector position, yaw;
    success =
        mav_planning_utils::polynomialTrajectoryMsgToPolynomialSegments<10>(
            msg, &position, &yaw);
    if (success) {
      trajectory_position->reset(
          new mav_planning_utils::PolynomialTrajectory<10>(3, position));
      trajectory_yaw->reset(
          new mav_planning_utils::PolynomialTrajectory<10>(1, yaw));
    }
  } else {
    ROS_WARN(
        "[trajectory sampler]: trajectory_msg must have num_coeffs=12 or "
        "num_coeffs=10, "
        "got num_coeffs=%d instead ",
        num_coeffs);
    return false;
  }
  return success;
}

template <int N_>
bool polynomialSegmentToPolynomialTrajectoryMsg(
    const typename Segment<N_>::Vector& position_segments,
    const typename Segment<N_>::Vector& yaw_segments,
    planning_msgs::PolynomialTrajectory4D* trajectory_msg) {
  CHECK_NOTNULL(trajectory_msg);
  trajectory_msg->segments.clear();

  bool success = true;

  if (position_segments.size() != yaw_segments.size()) {
    ROS_ERROR("Numbers of position and yaw segments are not equal: %lu / %lu.",
              position_segments.size(), yaw_segments.size());
    return false;
  }
  trajectory_msg->segments.reserve(position_segments.size());
  for (size_t i = 0; i < position_segments.size(); ++i) {
    const Segment<N_>& position_segment = position_segments[i];
    const Segment<N_>& yaw_segment = yaw_segments[i];
    if (position_segment.getTime() != yaw_segment.getTime()) {
      ROS_ERROR("Segment times have to be equal, but are: %f / %f.",
                position_segment.getTime(), yaw_segment.getTime());
      success = false;
      break;
    }

    if (position_segment.getDimension() != 3) {
      ROS_ERROR("Dimension of position segment has to be 3, but is %lu.",
                position_segment.getDimension());
      success = false;
      break;
    }

    if (yaw_segment.getDimension() != 1) {
      ROS_ERROR("Dimension of yaw segment has to be 1, but is %lu.",
                yaw_segment.getDimension());
      success = false;
      break;
    }

    planning_msgs::PolynomialSegment4D segment;
    planning_msgs::EigenPolynomialSegment eigen_segment;
    eigen_segment.x = position_segment[0].template getCoefficients<POSITION>();
    eigen_segment.y = position_segment[1].template getCoefficients<POSITION>();
    eigen_segment.z = position_segment[2].template getCoefficients<POSITION>();
    eigen_segment.yaw = yaw_segment[0].template getCoefficients<ORIENTATION>();
    eigen_segment.num_coeffs = N_;
    eigen_segment.segment_time_ns = position_segment.getTimeNSec();

    planning_msgs::polynomialSegmentMsgFromEigen(eigen_segment, &segment);

    trajectory_msg->segments.push_back(segment);
  }

  if (!success) trajectory_msg->segments.clear();

  return success;
}

template bool polynomialSegmentToPolynomialTrajectoryMsg<10>(
    const Segment<10>::Vector& position_segments,
    const Segment<10>::Vector& yaw_segments,
    planning_msgs::PolynomialTrajectory4D* trajectory_msg);

template bool polynomialSegmentToPolynomialTrajectoryMsg<12>(
    const Segment<12>::Vector& position_segments,
    const Segment<12>::Vector& yaw_segments,
    planning_msgs::PolynomialTrajectory4D* trajectory_msg);

template <int N_>
bool polynomialSegmentToPolynomialTrajectoryMsg(
    const typename Segment<N_>::Vector& position_segments,
    planning_msgs::PolynomialTrajectory4D* trajectory_msg) {
  CHECK_NOTNULL(trajectory_msg);
  trajectory_msg->segments.clear();

  bool success = true;

  trajectory_msg->segments.reserve(position_segments.size());
  for (size_t i = 0; i < position_segments.size(); ++i) {
    const Segment<N_>& position_segment = position_segments[i];

    if (position_segment.getDimension() != 3) {
      ROS_ERROR("Dimension of position segment has to be 3, but is %lu.",
                position_segment.getDimension());
      success = false;
      break;
    }

    planning_msgs::PolynomialSegment4D segment;
    planning_msgs::EigenPolynomialSegment eigen_segment;
    eigen_segment.x = position_segment[0].template getCoefficients<POSITION>();
    eigen_segment.y = position_segment[1].template getCoefficients<POSITION>();
    eigen_segment.z = position_segment[2].template getCoefficients<POSITION>();
    eigen_segment.num_coeffs = N_;
    eigen_segment.segment_time_ns = position_segment.getTimeNSec();

    planning_msgs::polynomialSegmentMsgFromEigen(eigen_segment, &segment);
    trajectory_msg->segments.push_back(segment);
  }

  if (!success) trajectory_msg->segments.clear();
  return success;
}

template bool polynomialSegmentToPolynomialTrajectoryMsg<10>(
    const Segment<10>::Vector& position_segments,
    planning_msgs::PolynomialTrajectory4D* trajectory_msg);

template bool polynomialSegmentToPolynomialTrajectoryMsg<12>(
    const Segment<12>::Vector& position_segments,
    planning_msgs::PolynomialTrajectory4D* trajectory_msg);

bool trajectoryToPolynomialTrajectoryMsg(
    TrajectoryBase::Ptr trajectory_position, TrajectoryBase::Ptr trajectory_yaw,
    planning_msgs::PolynomialTrajectory4D* trajectory_msg) {
  // We don't know the derived type and the number of coefficients in the base
  // class,
  // let's make a blind guess and hope for the best ...
  std::shared_ptr<PolynomialTrajectory<12>> polynomial_trajectory_position =
      std::dynamic_pointer_cast<PolynomialTrajectory<12>>(trajectory_position);

  if (polynomial_trajectory_position) {
    std::shared_ptr<PolynomialTrajectory<12>> polynomial_trajectory_yaw =
        std::dynamic_pointer_cast<PolynomialTrajectory<12>>(trajectory_yaw);

    if (!polynomial_trajectory_yaw) {
      ROS_WARN(
          "[trajectory sampler]: trajectory_yaw must have same num_coeffs");
      return false;
    }
    polynomialSegmentToPolynomialTrajectoryMsg<12>(
        polynomial_trajectory_position->getSegments(),
        polynomial_trajectory_yaw->getSegments(), trajectory_msg);
    return true;
  }

  // We don't know the derived type and the number of coefficients in the base
  // class,
  // let's make a blind guess and hope for the best ...
  std::shared_ptr<PolynomialTrajectory<10>> polynomial_trajectory_position_10 =
      std::dynamic_pointer_cast<PolynomialTrajectory<10>>(trajectory_position);
  if (polynomial_trajectory_position_10) {
    std::shared_ptr<PolynomialTrajectory<10>> polynomial_trajectory_yaw =
        std::dynamic_pointer_cast<PolynomialTrajectory<10>>(trajectory_yaw);
    if (!polynomial_trajectory_yaw) {
      ROS_WARN(
          "[trajectory sampler]: trajectory_yaw must have same num_coeffs");
      return false;
    }
    polynomialSegmentToPolynomialTrajectoryMsg<10>(
        polynomial_trajectory_position_10->getSegments(),
        polynomial_trajectory_yaw->getSegments(), trajectory_msg);
    return true;
  }

  ROS_WARN(
      "[trajectory sampler]: trajectory_position must be of"
      " type polynomialTrajectory and have num_coeffs=12 or num_coeffs=10");
  return false;
}

template bool polynomialTrajectoryToFile<10>(
    mav_planning_utils::TrajectoryBase::Ptr trajectory_position,
    mav_planning_utils::TrajectoryBase::Ptr trajectory_yaw,
    const std::string& filename);
template bool polynomialTrajectoryToFile<12>(
    mav_planning_utils::TrajectoryBase::Ptr trajectory_position,
    mav_planning_utils::TrajectoryBase::Ptr trajectory_yaw,
    const std::string& filename);

template <int N_>
bool polynomialTrajectoryToFile(
    mav_planning_utils::TrajectoryBase::Ptr trajectory_position,
    mav_planning_utils::TrajectoryBase::Ptr trajectory_yaw,
    const std::string& filename) {
  // We don't know the derived type and the number of coefficients in the base
  std::shared_ptr<mav_planning_utils::PolynomialTrajectory<N_>>
      polynomial_trajectory_position = std::dynamic_pointer_cast<
          mav_planning_utils::PolynomialTrajectory<N_>>(trajectory_position);
  std::shared_ptr<
      mav_planning_utils::PolynomialTrajectory<N_>> polynomial_trajectory_yaw =
      std::dynamic_pointer_cast<mav_planning_utils::PolynomialTrajectory<N_>>(
          trajectory_yaw);

  bool success = true;

  typename mav_planning_utils::Segment<N_>::Vector position_segments =
      polynomial_trajectory_position->getSegments();
  typename mav_planning_utils::Segment<N_>::Vector yaw_segments =
      polynomial_trajectory_yaw->getSegments();

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "num_segments" << YAML::Value << position_segments.size();
  out << YAML::Key << "segments" << YAML::Value;
  out << YAML::BeginSeq;

  for (size_t i = 0; i < position_segments.size(); ++i) {
    mav_planning_utils::Segment<N_>& position_segment = position_segments[i];
    mav_planning_utils::Segment<N_>& yaw_segment = yaw_segments[i];

    if (position_segment.getTime() != yaw_segment.getTime()) {
      ROS_ERROR("Segment times have to be equal, but are: %f / %f.",
                position_segment.getTime(), yaw_segment.getTime());
      success = false;
      break;
    }

    if (position_segment.getDimension() != 3) {
      ROS_ERROR("Dimension of position segment has to be 3, but is %lu.",
                position_segment.getDimension());
      success = false;
      break;
    }

    if (yaw_segment.getDimension() != 1) {
      ROS_ERROR("Dimension of yaw segment has to be 1, but is %lu.",
                yaw_segment.getDimension());
      success = false;
      break;
    }

    out << YAML::BeginMap;

    out << YAML::Key << "num_coeffs" << YAML::Value << N_;
    out << YAML::Key << "segment_time" << YAML::Value
        << position_segment.getTimeNSec() << YAML::Comment("[ns]");
    Eigen::Matrix<double, 1, N_> x =
        position_segment[0]
            .template getCoefficients<
                mav_planning_utils::derivative_order::POSITION>();

    out << YAML::Key << "x" << YAML::Value;
    out << YAML::Flow;
    out << YAML::BeginSeq;
    for (int i = 0; i < N_; ++i) {
      out << x[i];
    }
    out << YAML::EndSeq;
    Eigen::Matrix<double, 1, N_> y =
        position_segment[1]
            .template getCoefficients<
                mav_planning_utils::derivative_order::POSITION>();
    out << YAML::Key << "y" << YAML::Value;
    out << YAML::Flow;
    out << YAML::BeginSeq;
    for (int i = 0; i < N_; ++i) {
      out << y[i];
    }
    out << YAML::EndSeq;
    Eigen::Matrix<double, 1, N_> z =
        position_segment[2]
            .template getCoefficients<
                mav_planning_utils::derivative_order::POSITION>();
    out << YAML::Key << "z" << YAML::Value;
    out << YAML::Flow;
    out << YAML::BeginSeq;
    for (int i = 0; i < N_; ++i) {
      out << z[i];
    }
    out << YAML::EndSeq;
    Eigen::Matrix<double, 1, N_> yaw =
        yaw_segment[0]
            .template getCoefficients<
                mav_planning_utils::derivative_order::ORIENTATION>();
    out << YAML::Key << "yaw" << YAML::Value;
    out << YAML::Flow;
    out << YAML::BeginSeq;
    for (int i = 0; i < N_; ++i) {
      out << yaw[i];
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;

  if (success) {
    std::ofstream fout(filename);
    fout << out.c_str();
    fout.close();
  }

  return success;
}

template bool polynomialTrajectoryFromFile<10>(
    const std::string& filename,
    mav_planning_utils::TrajectoryBase::Ptr* trajectory_position,
    mav_planning_utils::TrajectoryBase::Ptr* trajectory_yaw);
template bool polynomialTrajectoryFromFile<12>(
    const std::string& filename,
    mav_planning_utils::TrajectoryBase::Ptr* trajectory_position,
    mav_planning_utils::TrajectoryBase::Ptr* trajectory_yaw);

template <int N_>
bool polynomialTrajectoryFromFile(
    const std::string& filename,
    mav_planning_utils::TrajectoryBase::Ptr* trajectory_position,
    mav_planning_utils::TrajectoryBase::Ptr* trajectory_yaw) {
  YAML::Node node = YAML::LoadFile(filename);

  // int num_segments = node["num_segments"].as<int>();

  typename mav_planning_utils::Segment<N_>::Vector segments_position;
  typename mav_planning_utils::Segment<N_>::Vector segments_yaw;

  mav_planning_utils::Segment<N_> position_segment(3);
  mav_planning_utils::Segment<N_> yaw_segment(1);

  const YAML::Node& segments = node["segments"];
  for (size_t i = 0; i < segments.size(); ++i) {
    const YAML::Node& segment = segments[i];

    double segment_time = segment["segment_time"].as<uint64_t>() * 1.0e-9;

    position_segment.setTime(segment_time);
    yaw_segment.setTime(segment_time);

    const YAML::Node& x_coeffs = segment["x"];
    Eigen::Matrix<double, 1, N_> coeffs;
    for (size_t i = 0; i < x_coeffs.size(); ++i) {
      const YAML::Node& x_coeff = x_coeffs[i];
      coeffs[i] = x_coeff.as<double>();
    }
    position_segment[0].setCoefficients(coeffs);

    const YAML::Node& y_coeffs = segment["y"];
    for (size_t i = 0; i < y_coeffs.size(); ++i) {
      const YAML::Node& y_coeff = y_coeffs[i];
      coeffs[i] = y_coeff.as<double>();
    }
    position_segment[1].setCoefficients(coeffs);

    const YAML::Node& z_coeffs = segment["z"];
    for (size_t i = 0; i < z_coeffs.size(); ++i) {
      const YAML::Node& z_coeff = z_coeffs[i];
      coeffs[i] = z_coeff.as<double>();
    }
    position_segment[2].setCoefficients(coeffs);

    const YAML::Node& yaw_coeffs = segment["yaw"];
    for (size_t i = 0; i < yaw_coeffs.size(); ++i) {
      const YAML::Node& yaw_coeff = yaw_coeffs[i];
      coeffs[i] = yaw_coeff.as<double>();
    }
    yaw_segment[0].setCoefficients(coeffs);

    segments_position.push_back(position_segment);
    segments_yaw.push_back(yaw_segment);
  }

  trajectory_position->reset(
      new mav_planning_utils::PolynomialTrajectory<N_>(3, segments_position));
  trajectory_yaw->reset(
      new mav_planning_utils::PolynomialTrajectory<N_>(1, segments_yaw));

  return true;
}

}  // namespace mav_planning_utils
