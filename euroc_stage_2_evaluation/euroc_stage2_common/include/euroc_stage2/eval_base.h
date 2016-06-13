#ifndef EVAL_BASE_H
#define EVAL_BASE_H

#include <Eigen/Eigen>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <euroc_stage2/results_file_writer.h>

namespace euroc_stage2 {

class EvalBase {
 public:
  EvalBase(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh,
           const std::string task_name);

 private:
  ResultsFileWriter readResultsWriterParams(const std::string task_name);

  void saverConstraintsViolatedFlagCallback(const std_msgs::BoolConstPtr& msg);

  // subscribers
  ros::Subscriber saver_constraints_violated_flag_sub_;

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ResultsFileWriter results_writer_;
  bool saver_constraints_violated_flag_;
};

// Default values
constexpr double kQueueSize = 1000;
const std::string kDefaultTeamName = "ETH";
const std::string kDefaultResultsPath = "../results";
}
#endif