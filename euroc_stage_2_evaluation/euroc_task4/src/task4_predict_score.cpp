#include <octomap_ros/conversions.h>
#include <visualization_msgs/MarkerArray.h>

#include <euroc_stage2/trajectory_interface.h>
#include <euroc_stage2/csv_parser.h>

#include <euroc_stage2/task4_predict_score.h>

namespace euroc_stage2 {

Task4PredictScore::Task4PredictScore(const ros::NodeHandle& nh,
                                     const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), num_obstacles_(5) {
  std::string waypoint_file = "task4_waypoints.txt";
  std::string results_file = "task4_results.txt";

  nh_private_.param("waypoint_file", waypoint_file, waypoint_file);
  nh_private_.param("results_file", results_file, results_file);

  nh_private_.param("num_obstacles", num_obstacles_, num_obstacles_);

  if (!readWaypointsFromCsv(waypoint_file) || waypoint_positions_.empty() ||
      waypoint_times_sec_.empty()) {
    ROS_FATAL("Could not read waypoint file!");
    ros::shutdown();
    return;
  }

  if (!readResultsFromCsv(results_file) || results_file.empty()) {
    ROS_FATAL("Could not read results file!");
    ros::shutdown();
    return;
  }
}

Task4PredictScore::~Task4PredictScore() {}

bool Task4PredictScore::readWaypointsFromCsv(const std::string& filename) {
  const size_t num_columns = 4;  // time, then 3 for position.
  std::vector<Eigen::VectorXd> parsed_vector;

  bool success = parseCsvIntoVector(filename, num_columns, &parsed_vector);

  waypoint_times_sec_.clear();
  waypoint_positions_.clear();
  if (success && !parsed_vector.empty()) {
    waypoint_times_sec_.resize(parsed_vector.size());
    waypoint_positions_.resize(parsed_vector.size());

    for (size_t i = 0; i < parsed_vector.size(); ++i) {
      waypoint_times_sec_[i] = parsed_vector[i](0);
      waypoint_positions_[i] = Eigen::Vector3d(
          parsed_vector[i](1), parsed_vector[i](2), parsed_vector[i](3));
    }
  }

  return success;
}

bool Task4PredictScore::readResultsFromCsv(const std::string& filename) {
  const size_t num_columns = 14;
  //  tstamp_ns, pos_x, pos_y, pos_z, yaw, vel_x, vel_y, vel_z,
  //  yaw_rate, cvel_x, cvel_y, cvel_z, cyaw_rate, dist
  std::vector<Eigen::VectorXd> parsed_vector;

  bool success = parseCsvIntoVector(filename, num_columns, &parsed_vector);
  ROS_INFO("Did it work? %d", success);

  results_.clear();
  uint64_t initial_time = 0;
  const double kNSecToSec = 1e-9;
  if (success && !parsed_vector.empty()) {
    for (size_t i = 0; i < parsed_vector.size(); ++i) {
      if (i == 0) {
        initial_time = parsed_vector[i](0);
      }
      results_.push_back(std::pair<double, double>(
          kNSecToSec * (parsed_vector[i](0) - initial_time),
          parsed_vector[i](13)));
    }
  }

  return success;
}

double Task4PredictScore::predictScore() {
  // Go through all waypoints in the vector.
  std::vector<double> min_dist_per_waypoint(waypoint_times_sec_.size());

  ROS_INFO("[Minimum distances by waypoint]:\n");
  size_t j = 0;  // Index within the results.
  double last_waypoint_time = 0.0;
  for (size_t i = 0; i < waypoint_times_sec_.size(); ++i) {
    if (j >= results_.size()) {
      break;
    }
    min_dist_per_waypoint[i] = results_[j].second;
    for (; results_[j].first - last_waypoint_time < waypoint_times_sec_[i] &&
           j < results_.size();
         ++j) {
      if (results_[j].second < min_dist_per_waypoint[i]) {
        min_dist_per_waypoint[i] = results_[j].second;
      }
    }
    last_waypoint_time += waypoint_times_sec_[i];
    ROS_INFO("\t[Waypoint %lu]: %f, j: %lu out of %lu", i,
             min_dist_per_waypoint[i], j, results_.size());
  }

  ROS_INFO("Selecting the smallest %d distances (rest should be free space).",
           num_obstacles_);

  std::sort(min_dist_per_waypoint.begin(), min_dist_per_waypoint.end());

  // TODO(helenol): hardcode these. Whatever.
  double score = 0.0;
  for (int k = 0; k < num_obstacles_ && k < min_dist_per_waypoint.size(); ++k) {
    double dist = min_dist_per_waypoint[k];
    if (dist < 0.8) {
      score += 0.0;
    } else if (dist < 1.2) {
      score += 0.3;
    } else if (dist < 1.6) {
      score += 1.0;
    } else if (dist < 2.0) {
      score += 0.5;
    } else {
      score += 0.0;
    }
  }

  score /= num_obstacles_;

  ROS_INFO("Final score: %f%% out of 100%%", score * 100.0);

  return score;
}

}  // namespace euroc_stage2

int main(int argc, char** argv) {
  ros::init(argc, argv, "task4_predict_score");

  ros::NodeHandle nh, private_nh("~");

  ROS_INFO("Started task4 score predictor node.");

  euroc_stage2::Task4PredictScore task4_predict_score(nh, private_nh);
  ros::spinOnce();
  task4_predict_score.predictScore();
  ros::spinOnce();
  return 0;
}
