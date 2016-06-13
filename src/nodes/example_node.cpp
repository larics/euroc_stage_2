/*
* Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
* You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#include <ros/ros.h>

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_planning_utils/polynomial_optimization_nonlinear.h>
#include <mav_planning_utils/polynomial_trajectory.h>
#include <mav_planning_utils/ros_trajectory_interface.h>

using namespace mav_planning_utils;
using namespace visualization_msgs;

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_example_node");
  ros::NodeHandle nh;
  ros::Publisher path_pub = nh.advertise<MarkerArray>("path", 10);

  // generate test data
  Eigen::Vector3d pos_min(-5, -5, 0), pos_max(5, 5, 5);
  const int N = 10;
  const int derivative_to_optimize = derivative_order::SNAP;

  Vertex::Vector vertices =
      createRandomVertices(derivative_order::SNAP, 5, pos_min, pos_max, 70983);
  const double approximate_v_max = 4.0;
  const double approximate_a_max = 5.0;
  std::vector<double> segment_times = estimateSegmentTimes(
      vertices, approximate_v_max / 4, approximate_a_max / 4);

  // setup parameters for nlopt
  NonlinearOptimizationParameters parameters;
  parameters.max_iterations = 1000;
  parameters.f_rel = 0.05;
  parameters.x_rel = 0.1;
  parameters.time_penalty = 500.0;
  parameters.initial_stepsize_rel = 0.1;
  parameters.inequality_constraint_tolerance = 0.1;
  //  parameters.algorithm = nlopt::GN_ORIG_DIRECT;
  //  parameters.algorithm = nlopt::GN_ORIG_DIRECT_L;
  parameters.algorithm = nlopt::GN_ISRES;
  //  parameters.algorithm = nlopt::LN_COBYLA;
  parameters.random_seed = 12345678;

  // instantiate nlopt, add inequality constraints and run.
  PolynomialOptimizationNonLinear<N> nl_optimizer(3, parameters, false);
  nl_optimizer.setupFromVertices(vertices, segment_times,
                                 derivative_to_optimize);
  nl_optimizer.addMaximumMagnitudeConstraint(derivative_order::VELOCITY,
                                             approximate_v_max);
  nl_optimizer.addMaximumMagnitudeConstraint(derivative_order::ACCELERATION,
                                             approximate_a_max);
  int ret = nl_optimizer.optimize();

  std::cout << "nlopt stopped for reason: " << nlopt::returnValueToString(ret)
            << std::endl;

  // Get underlying linear problem, and obtain segments.
  // TODO(acmarkus) we may wan to have a nicer interface for this ;)
  PolynomialOptimization<N> linear_problem(
      nl_optimizer.getPolynomialOptimizationRef());
  Segment<N>::Vector segments;
  linear_problem.getSegments(&segments);
  linear_problem.getSegmentTimes(&segment_times);

  // Get some stats.
  Extremum v_max =
      linear_problem.computeMaximumOfMagnitude<derivative_order::VELOCITY>(
          nullptr);
  Extremum a_max =
      linear_problem.computeMaximumOfMagnitude<derivative_order::ACCELERATION>(
          nullptr);
  std::cout << "v_max: " << v_max.value << " a_max: " << a_max.value
            << std::endl;
  std::cout << "segments" << segments.size() << "   "
            << segments.back().getTime() << std::endl;

  // Create a trajectory from the vector of segments.
  PolynomialTrajectory<N> trajectory(3, segments);

  // Sample trajectory to flat states, then draw.
  mav_msgs::EigenTrajectoryPoint::Vector flat_states;
  sampleWholeTrajectory(trajectory, 0.1, &flat_states);
  mav_viz::HexacopterMarker hex;
  MarkerArray markers;
  drawMavTrajectory(flat_states, hex, 1.0, &markers);

  while (ros::ok()) {
    path_pub.publish(markers);
    std::cout << "published " << markers.markers.size() << std::endl;
    ros::Duration(5.0).sleep();
  }

  return EXIT_SUCCESS;
}
