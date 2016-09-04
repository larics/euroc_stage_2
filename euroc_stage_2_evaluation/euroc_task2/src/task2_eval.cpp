#include <euroc_stage2/task2_eval.h>

namespace euroc_stage2 {

Task2Eval::Task2Eval(const ros::NodeHandle& nh,
                     const ros::NodeHandle& private_nh)
    : EvalBase(nh, private_nh, "Task 2") {
  pose_sub_ = nh_.subscribe("pose", kQueueSize, &Task2Eval::poseCallback, this);

  bool wind_trajectory;
  private_nh.param("wind_trajectory", wind_trajectory, kDefaultIsWindTrajectory);

  if(wind_trajectory){
    trajectory_evaluators_.emplace_back(TrajectoryEvaluator(0, kWindSubTaskLength));
    marker_pubs_.emplace_back(nh_.advertise<visualization_msgs::MarkerArray>("path_aligned_wind", 0));
  }
  else{
    for (size_t i = 0; i < kNumSubTasks; ++i) {
      trajectory_evaluators_.emplace_back(
          TrajectoryEvaluator(kSubTaskLength * i, kSubTaskLength * (i + 1)));
      
      marker_pubs_.emplace_back(nh_.advertise<visualization_msgs::MarkerArray>("path_aligned_" + std::to_string(i), 0));
    }
  }

  trajectory_sub_ = nh_.subscribe(
        "command/trajectory", kQueueSize,
        &Task2Eval::trajectoryCallback, this);
}

void Task2Eval::trajectoryCallback(
    trajectory_msgs::MultiDOFJointTrajectoryConstPtr msg){
      for (TrajectoryEvaluator& traj_eval : trajectory_evaluators_){
        traj_eval.trajectoryCallback(msg);
      }
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
  bool new_best = false;
  for(TrajectoryEvaluator& traj_eval : trajectory_evaluators_){
    if (traj_eval.addPoint(point_time, point_position)) {
      new_best = true;
    }
  }

  //write results
  if(new_best){
    writeTime();
    results_writer_ << "RMS Error of [";
    for(TrajectoryEvaluator& traj_eval : trajectory_evaluators_){
      results_writer_ << traj_eval.getMinError() << " ";
    }
    results_writer_ << "], at a time offset of [";
    for(TrajectoryEvaluator& traj_eval : trajectory_evaluators_){
      results_writer_ << traj_eval.getTimeOffsetFromStart() << " ";
    }
    results_writer_ << "]\n";
  }

  //update markers
  for(size_t i = 0; i < trajectory_evaluators_.size(); ++i){
    marker_pubs_[i].publish(trajectory_evaluators_[i].getMarkers());
  }
}
};
