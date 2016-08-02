#include <euroc_stage2/waypoint.h>

namespace euroc_stage2 {

bool getAbsoluteFilePath(const std::string& package_name,
                         std::string* file_name) {
  // Only do something if it is not already an absolute path.
  if (file_name->at(0) != '/') {
    std::string package_path = ros::package::getPath(package_name);
    std::stringstream ss;
    ss.str("");
    ss << package_path << '/' << *file_name;
    *file_name = ss.str();
    return true;
  }
  return false;
}

enum MarkerIdx { MARK_IDX = 0, TEXT_IDX };

Waypoint::Waypoint(const Eigen::Vector3d& position, double radius,
                   double required_wait_duration, const ros::Time& start_time)
    : position_(position),
      radius_(radius),
      required_wait_duration_(required_wait_duration),
      start_time_(start_time),
      finish_duration_(0, 0),
      first_update_(true) {
  visualization_msgs::Marker marker_waypoint;
  marker_waypoint.header.frame_id = "world";
  marker_waypoint.id = MARK_IDX;
  marker_waypoint.type = visualization_msgs::Marker::SPHERE;
  marker_waypoint.action = visualization_msgs::Marker::ADD;
  marker_waypoint.pose.position.x = position_[0];
  marker_waypoint.pose.position.y = position_[1];
  marker_waypoint.pose.position.z = position_[2];
  marker_waypoint.scale.x = radius_ * 2;
  marker_waypoint.scale.y = radius_ * 2;
  marker_waypoint.scale.z = radius_ * 2;
  marker_waypoint.color.a = 0.5;
  marker_waypoint.color.r = 1.0;
  marker_waypoint.color.g = 0.0;
  marker_waypoint.color.b = 0.0;

  markerArray_.markers.push_back(marker_waypoint);

  visualization_msgs::Marker marker_text;
  marker_text.header.frame_id = "world";
  marker_text.id = TEXT_IDX;
  marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_text.action = visualization_msgs::Marker::ADD;
  marker_text.pose.position.x = position_[0];
  marker_text.pose.position.y = position_[1];
  marker_text.pose.position.z = position_[2] + radius_;
  marker_text.scale.z = 0.2;
  marker_text.color.a = 1.0;
  marker_text.color.r = 1.0;
  marker_text.color.g = 1.0;
  marker_text.color.b = 1.0;
  marker_text.text = "";

  markerArray_.markers.push_back(marker_text);
}

const visualization_msgs::MarkerArray& Waypoint::getMarkers(void) {
  return markerArray_;
}

bool Waypoint::updateStatus(const Eigen::Vector3d& current_position,
                            const ros::Time& current_time, bool valid) {
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2);

  if (first_update_) {
    time_when_entered_ = current_time;
    first_update_ = false;
  }

  double dist = (current_position - position_).lpNorm<2>();
  ros::Duration duration_rem =
      required_wait_duration_ - (current_time - time_when_entered_);
  ros::Duration duration_elapsed = current_time - start_time_;

  if (dist < radius_) {
    if ((duration_rem.toSec() <= 0.0) && (finish_duration_.toSec() <= 0.0))
      finish_duration_ = duration_elapsed;

    markerArray_.markers[MARK_IDX].color.r = 0.0;
    markerArray_.markers[MARK_IDX].color.g = 1.0;
    markerArray_.markers[MARK_IDX].color.b = 0.0;

  } else {
    time_when_entered_ = current_time;
    duration_rem = required_wait_duration_;
    markerArray_.markers[MARK_IDX].color.g = 0.0;
    markerArray_.markers[MARK_IDX].color.r = 1.0;
    markerArray_.markers[MARK_IDX].color.b = 0.0;
  }

  if (finish_duration_.toSec() > 0.0) {
    markerArray_.markers[MARK_IDX].color.r = 0.0;
    markerArray_.markers[MARK_IDX].color.g = 0.0;
    markerArray_.markers[MARK_IDX].color.b = 1.0;
    ss << "Waypoint reached in: " << finish_duration_.toSec();
    markerArray_.markers[TEXT_IDX].text = ss.str();
    return true;
  }

  if (!valid) {
    time_when_entered_ = current_time;
    duration_rem = required_wait_duration_;
    markerArray_.markers[MARK_IDX].color.r = 0.0;
    markerArray_.markers[MARK_IDX].color.g = 0.0;
    markerArray_.markers[MARK_IDX].color.b = 0.0;

    ss << "Waypoint invalid\n";
    markerArray_.markers[TEXT_IDX].text = ss.str();

    return false;
  }

  ss << "Distance: " << dist << "\n"
     << "Hold time remaining: " << duration_rem.toSec() << "\n"
     << "Time elapsed: " << duration_elapsed.toSec() << "\n";
  markerArray_.markers[TEXT_IDX].text = ss.str();

  return false;
}

bool Waypoint::updateStatus(const geometry_msgs::PoseStampedConstPtr& msg,
                            bool valid) {
  Eigen::Vector3d position(msg->pose.position.x, msg->pose.position.y,
                           msg->pose.position.z);

  return updateStatus(position, msg->header.stamp, valid);
}

bool Waypoint::updateStatus(const geometry_msgs::TransformStampedConstPtr& msg,
                            bool valid) {
  Eigen::Vector3d position(msg->transform.translation.x, msg->transform.translation.y,
                           msg->transform.translation.z);

  return updateStatus(position, msg->header.stamp, valid);
}

void Waypoint::clearFinishTime(void) { finish_duration_ = ros::Duration(0, 0); }

ros::Duration Waypoint::getFinishTime(void) { return finish_duration_; }

void Waypoint::changeStartTime(const ros::Time& start_time) {
  start_time_ = start_time;
}

}
