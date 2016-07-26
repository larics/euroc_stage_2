#include <euroc_stage2/eval_base.h>

#define QUEUE_SIZE 1000

namespace euroc_stage2 {

EvalBase::EvalBase(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh,
                   const std::string task_name)
    : nh_(nh),
      private_nh_(private_nh),
      results_writer_(readResultsWriterParams(task_name)),
      saver_constraints_violated_flag_(false) {
  saver_constraints_violated_flag_sub_ =
      nh_.subscribe("saver_constraints_violated", QUEUE_SIZE,
                    &EvalBase::saverConstraintsViolatedFlagCallback, this);

  writeTime();
  results_writer_ << "Starting " << task_name << "\n";
}

void EvalBase::writeTime(){
  results_writer_ << "[" <<ros::Time::now() << "]: ";
}

ResultsFileWriter EvalBase::readResultsWriterParams(
    const std::string task_name) {
  std::string team_name, results_path;
  private_nh_.param("team_name", team_name, kDefaultTeamName);
  private_nh_.param("results_path", results_path, kDefaultResultsPath);
  return ResultsFileWriter(results_path, team_name, task_name);
}

void EvalBase::saverConstraintsViolatedFlagCallback(
    const std_msgs::BoolConstPtr& msg) {
  saver_constraints_violated_flag_ = msg->data;
}
};
