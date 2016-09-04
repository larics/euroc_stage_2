#include <euroc_stage2/trajectory_evaluator.h>

namespace euroc_stage2 {

enum MarkerIdx { MARK_IDX = 0, TEXT_IDX };

TrajectoryEvaluator::TrajectoryEvaluator(double trajectory_start_distance,
                                         double trajectory_end_distance)
    : trajectory_start_distance_(trajectory_start_distance),
      trajectory_end_distance_(trajectory_end_distance),
      min_rms_error_(std::numeric_limits<double>::max()),
      time_offset_(0) {
  visualization_msgs::Marker marker_trajectory;
  marker_trajectory.header.frame_id = "vicon";
  marker_trajectory.id = MARK_IDX;
  marker_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
  marker_trajectory.action = visualization_msgs::Marker::ADD;
  marker_trajectory.pose.position.x = 0.0;
  marker_trajectory.pose.position.y = 0.0;
  marker_trajectory.pose.position.z = 0.0;
  marker_trajectory.scale.x = 0.1;
  marker_trajectory.color.a = 0.5;
  marker_trajectory.color.r = 0.0;
  marker_trajectory.color.g = 1.0;
  marker_trajectory.color.b = 0.0;

  marker_array_.markers.push_back(marker_trajectory);

  visualization_msgs::Marker marker_text;
  marker_text.header.frame_id = "vicon";
  marker_text.id = TEXT_IDX;
  marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_text.action = visualization_msgs::Marker::ADD;
  marker_text.pose.position.x = 0.0;
  marker_text.pose.position.y = 0.0;
  marker_text.pose.position.z = 0.0;
  marker_text.scale.z = 0.2;
  marker_text.color.a = 1.0;
  marker_text.color.r = 0.0;
  marker_text.color.g = 1.0;
  marker_text.color.b = 0.0;
  marker_text.text = "No valid trajectory found";

  marker_array_.markers.push_back(marker_text);
}

void TrajectoryEvaluator::trajectoryCallback(
    trajectory_msgs::MultiDOFJointTrajectoryConstPtr msg) {
  desired_path_.clear();

  double trajectory_length = 0.0;

  Eigen::Vector3d prev_position;
  prev_position.x() = msg->points.front().transforms.front().translation.x;
  prev_position.y() = msg->points.front().transforms.front().translation.y;
  prev_position.z() = msg->points.front().transforms.front().translation.z;

  // extract points and times
  for (auto& i : msg->points) {
    std::pair<ros::Duration, Eigen::Vector3d> point;
    point.first = i.time_from_start;
    point.second.x() = i.transforms.front().translation.x;
    point.second.y() = i.transforms.front().translation.y;
    point.second.z() = i.transforms.front().translation.z;
    trajectory_length += (point.second - prev_position).norm();
    prev_position = point.second;

    if ((trajectory_length >= trajectory_start_distance_) &&
        (trajectory_length < trajectory_end_distance_)) {
      desired_path_.push_back(point);
    }
  }

  if (!desired_path_.empty() && !(msg->points.empty())) {
    min_time_needed_ =
        desired_path_.back().first - msg->points.front().time_from_start;
    ROS_INFO_STREAM("Time length of trajectory: " << min_time_needed_);
  } else {
    ROS_WARN("Empty trajectory section detected");
  }
}

bool TrajectoryEvaluator::addPoint(const ros::Duration& point_time,
                                   const Eigen::Vector3d& point_position) {
  if (!flown_path_.empty() && flown_path_.back().first > point_time) {
    ROS_ERROR(
        "New pose has timestamp smaller than current last pose and will be "
        "ignored");
    return false;
  }

  flown_path_.push_back(std::make_pair(point_time, point_position));

  if (desired_path_.empty()) return false;

  // check if trajectory is long enough to evaluate
  ros::Duration flown_duration =
      flown_path_.back().first - flown_path_.front().first;
  ros::Duration desired_duration =
      desired_path_.back().first - desired_path_.front().first;
  if ((flown_duration < min_time_needed_) ||
      (flown_duration < desired_duration)) {
    return false;
  }
  std::pair<double, size_t> error_info = calcSquaredError();
  double rms_error = std::sqrt(error_info.first / error_info.second);
  // ROS_ERROR_STREAM(" " << error_info.first << " " << error_info.second);
  if (rms_error < min_rms_error_) {
    min_rms_error_ = rms_error;
    time_offset_ = flown_path_.back().first - desired_path_.back().first;
    updateMarkerPath(error_info.second);
    return true;
  } else {
    return false;
  }
}

visualization_msgs::MarkerArray TrajectoryEvaluator::getMarkers() const {
  return marker_array_;
}

double TrajectoryEvaluator::getMinError() const { return min_rms_error_; }

ros::Duration TrajectoryEvaluator::getTimeOffsetFromStart() const {
  if (flown_path_.size()) {
    return time_offset_ - flown_path_.front().first;
  } else {
    ROS_ERROR("Requested time offset for an empty path");
    return ros::Duration(0);
  }
}

void TrajectoryEvaluator::updateMarkerPath(size_t num_trajectory_points) {
  marker_array_.markers[MARK_IDX].points.clear();
  for (size_t i = flown_path_.size() - num_trajectory_points;
       i < flown_path_.size(); ++i) {
    geometry_msgs::Point point;
    point.x = flown_path_[i].second.x();
    point.y = flown_path_[i].second.y();
    point.z = flown_path_[i].second.z();
    marker_array_.markers[MARK_IDX].points.push_back(point);
  }

  marker_array_.markers[TEXT_IDX].pose.position =
      marker_array_.markers[MARK_IDX].points.back();
  std::stringstream marker_text;
  marker_text << std::setprecision(4) << "Trajectory time offset: "
              << (time_offset_ - flown_path_.front().first).toSec()
              << "\nRMS Error: " << min_rms_error_;
  ROS_INFO_STREAM(marker_text.str());
  marker_array_.markers[TEXT_IDX].text = marker_text.str();
}

// finds error when desired trajectory and flown trajectory finish at the same
// time
std::pair<double, size_t> TrajectoryEvaluator::calcSquaredError() const {
  ros::Duration current_time_offset =
      flown_path_.back().first - desired_path_.back().first;
  ros::Duration start_time =
      flown_path_.back().first - desired_path_.back().first;

  int index_offset = flown_path_.size() - desired_path_.size();

  double squared_error = 0;

  size_t i = 0;

  while ((i < desired_path_.size()) &&
         ((i + index_offset) < flown_path_.size())) {
    // skip points with missing vicon info
    if ((flown_path_[i + index_offset].first - desired_path_[i].first -
         start_time).toSec() > kVicondt) {
      --index_offset;
      ++i;
    }
    // skip eval if too many vicon messages
    else if ((flown_path_[i + index_offset].first - desired_path_[i].first -
              start_time).toSec() < -kVicondt) {
      ++index_offset;
    } else {
      Eigen::Vector3d diff =
          flown_path_[i + index_offset].second - desired_path_[i].second;
      squared_error += diff.squaredNorm();
      ++i;
    }
  }
  return std::make_pair(squared_error, i);
}
}
