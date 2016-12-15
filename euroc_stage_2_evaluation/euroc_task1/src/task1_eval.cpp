#include <euroc_stage2/task1_eval.h>

namespace euroc_stage2 {

Task1Eval::Task1Eval(const ros::NodeHandle& nh,
                     const ros::NodeHandle& private_nh)
    : EvalBase(nh, private_nh, "task1"), waypoint_(nullptr) {
  start_srv_ = private_nh_.advertiseService(
      "start_evaluation", &Task1Eval::startTaskEvaluationCallback, this);

  vis_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("waypoint_marker", 0);

  vicon_sub_ = nh_.subscribe("vicon_transform", kQueueSize,
                             &Task1Eval::viconUpdateCallback, this);

  finished_ = false;
}

Waypoint Task1Eval::readWaypointParams() {
  Eigen::Vector3d pos;
  double radius;
  double hold_time;
  private_nh_.param("position/x", pos.x(), kDefaultWaypointPositionX);
  private_nh_.param("position/y", pos.y(), kDefaultWaypointPositionY);
  private_nh_.param("position/z", pos.z(), kDefaultWaypointPositionZ);
  private_nh_.param("radius", radius, kDefaultWaypointRadius);
  private_nh_.param("hold_time", hold_time, kDefaultWaypointHoldTime);

  private_nh_.param("task2_service_name", task2_service_name_, kDefaultTask2ServiceName);

  // kill time until clock message received (only needed for simulation)
  while (ros::Time::now().toSec() < kSmallTime) continue;

  return Waypoint(pos, radius, hold_time, ros::Time::now());
}

bool Task1Eval::startTaskEvaluationCallback(std_srvs::EmptyRequest& request,
                                            std_srvs::EmptyResponse& response) {
  waypoint_ = std::make_shared<Waypoint>(readWaypointParams());
  if (waypoint_)
    return true;
  else
    return false;
}

void Task1Eval::viconUpdateCallback(
    const geometry_msgs::TransformStampedConstPtr& msg) {
  if (waypoint_) {
    bool finish_update =
        waypoint_->updateStatus(msg, !saver_constraints_violated_flag_);
    if (!finished_ && finish_update) {
      writeTime();
      results_writer_ << "Waypoint reached in " << waypoint_->getFinishTime()
                      << " Seconds \n";

      //start task 2
      std_srvs::Empty::Request request;
      std_srvs::Empty::Response response;
      if(!ros::service::call(task2_service_name_, request, response)){
        ROS_ERROR("Failed to start task2 server");
      }

    }
    finished_ = finish_update;
    vis_pub_.publish(waypoint_->getMarkers());
  }
}
};
