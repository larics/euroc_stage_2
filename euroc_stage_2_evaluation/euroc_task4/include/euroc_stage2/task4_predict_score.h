#ifndef EUROC_TASK4_TASK4_PREDICT_SCCORE_H
#define EUROC_TASK4_TASK4_PREDICT_SCCORE_H

#include <stdio.h>
#include <Eigen/Eigen>
#include <iostream>
#include <string>

// ros
#include <ros/ros.h>

#include <mav_msgs/default_topics.h>
#include <mav_msgs/conversions.h>
#include <euroc_stage2/eval_base.h>

namespace euroc_stage2 {

// Horrible hack on top of task4eval.
class Task4PredictScore {
 public:
  Task4PredictScore(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private);
  virtual ~Task4PredictScore();

  bool readWaypointsFromCsv(const std::string& filename);
  bool readResultsFromCsv(const std::string& filename);

  double predictScore();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Number of obstacles to count -- will make sure all others are free space.
  int num_obstacles_;

  // Some kind of list of waypoints...
  std::vector<Eigen::Vector3d> waypoint_positions_;
  std::vector<double> waypoint_times_sec_;

  // Pair of time (s) to distance from obstacle.
  std::vector<std::pair<double, double> > results_;
};

}  // namespace euroc_stage2

#endif  // EUROC_TASK4_TASK4_PREDICT_SCCORE_H
