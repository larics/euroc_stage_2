#include <euroc_stage2/task2_eval.h>

namespace euroc_stage2 {

Task2Eval::Task2Eval(const ros::NodeHandle& nh,
                     const ros::NodeHandle& private_nh)
    : EvalBase(nh, private_nh, "Task 2") {
  pose_sub_ = nh_.subscribe("pose", kQueueSize, &Task2Eval::poseCallback, this);
  trajectory_sub_ = nh_.subscribe("command/trajectory", kQueueSize,
                                  &TrajectoryEvaluator::trajectoryCallback,
                                  &trajectory_evaluator_);
  marker_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("path_aligned", 0);
}

void Task2Eval::poseCallback(
    const geometry_msgs::TransformStampedConstPtr& msg) {
  // do nothing if saver in control
  if (saver_constraints_violated_flag_) return;

  Eigen::Vector3d point_position(msg->transform.translation.x,
                                 msg->transform.translation.y,
                                 msg->transform.translation.z);
  ros::Duration point_time(msg->header.stamp.sec, msg->header.stamp.nsec);

  // if new point improves metric score
  if (trajectory_evaluator_.addPoint(point_time, point_position)) {
    writeTime();
    results_writer_ << ros::Time::now() << ", RMS Error of "
                    << trajectory_evaluator_.getMinError()
                    << " at a time offset of "
                    << trajectory_evaluator_.getTimeOffsetFromStart() << "\n";
  }
  marker_pub_.publish(trajectory_evaluator_.getMarkers());
}
};
