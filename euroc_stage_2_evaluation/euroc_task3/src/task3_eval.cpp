#include <euroc_stage2/task3_eval.h>

namespace euroc_stage2 {

Task3Eval::Task3Eval(const ros::NodeHandle& nh,
                     const ros::NodeHandle& private_nh)
    : EvalBase(nh, private_nh, "Task 3") {
  transform_sub_ = nh_.subscribe("vrpn_client/pose", kQueueSize,
                                 &Task3Eval::poseCallback, this);

  waypoint_sub_ = nh_.subscribe(mav_msgs::default_topics::COMMAND_POSE,
                                kQueueSize, &Task3Eval::waypointCallback, this);

  half_score_srv_ = private_nh_.advertiseService(
      "half_score", &Task3Eval::halfScoreSrvs, this);

}

void Task3Eval::waypointCallback(const geometry_msgs::PoseStamped& msg) {
  Eigen::Vector3d position(msg.pose.position.x, msg.pose.position.y,
                           msg.pose.position.z);
  waypoint_.push_back(
      Waypoint(position, kWaypointRadius, kWaypointHoldTime, msg.header.stamp));
}

bool Task3Eval::halfScoreSrvs(std_srvs::EmptyRequest& request,
                                            std_srvs::EmptyResponse& response) {
  writeTime();
  results_writer_ << "VICON USED: SCORE MUST BE HALVED\n";
  return true;
}

void Task3Eval::poseCallback(
    const geometry_msgs::TransformStampedConstPtr& msg) {
  for (auto i = waypoint_.begin(); i != waypoint_.end();) {
    bool finished = i->updateStatus(msg, !saver_constraints_violated_flag_);
    // remove finished waypoints from vector
    if (finished) {
      writeTime();
      results_writer_ << "Waypoint reached in " << i->getFinishTime()
                      << " Seconds \n";
      i = waypoint_.erase(i);
    } else
      ++i;
  }
}
}
