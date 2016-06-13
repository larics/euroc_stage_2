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

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <planning_msgs/PolynomialTrajectory4D.h>
#include <planning_msgs/PolynomialSegment4D.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mav_planning_utils/polynomial_trajectory.h>
#include <mav_planning_utils/ros_trajectory_interface.h>

ros::Timer publish_timer;
ros::Publisher command_publisher;
ros::Publisher old_command_publisher;
std::shared_ptr<mav_planning_utils::TrajectoryBase> trajectory_position;
std::shared_ptr<mav_planning_utils::TrajectoryBase> trajectory_yaw;
double dt = 0;
bool publish_whole_trajectory = false;
double current_sample_time = 0;
ros::Time start_time;

void trajectoryCallback(const planning_msgs::PolynomialTrajectory4D& msg) {
  if (msg.segments.empty()) {
    ROS_WARN("Received empty waypoint message");
    return;
  } else
    ROS_INFO("Received %lu waypoints", msg.segments.size());

  bool success = mav_planning_utils::polynomialTrajectoryMsgToTrajectoryBase(
      msg, &trajectory_position, &trajectory_yaw);

  if (!success) {
    return;
  }

  if(publish_whole_trajectory) {
    // publish whole trajecory at once.
    mav_msgs::EigenTrajectoryPoint::Vector flat_states;
    mav_planning_utils::sampleWholeTrajectory(
        *trajectory_position, *trajectory_yaw, dt, &flat_states);

    trajectory_msgs::MultiDOFJointTrajectory msg_pub;
    msgMultiDofJointTrajectoryFromEigen(flat_states, &msg_pub);

    command_publisher.publish(msg_pub);
  } else {
    publish_timer.start();
    current_sample_time = 0;
    start_time = ros::Time::now();  // TODO(acmarkus) is that the best thing to
    // do?
  }
}

bool stopCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res) {
  publish_timer.stop();
  return true;
}

inline geometry_msgs::Vector3 eigenToVector3(const Eigen::Vector3d& v) {
  geometry_msgs::Vector3 msg;
  msg.x = v.x();
  msg.y = v.y();
  msg.z = v.z();
  return msg;
}

void timerCallback(const ros::TimerEvent& e) {
  if (current_sample_time <= trajectory_position->getMaxTime()) {
    trajectory_msgs::MultiDOFJointTrajectory msg;
    mav_msgs::EigenTrajectoryPoint flat_state;
    mav_planning_utils::sampleTrajectory(*trajectory_position, *trajectory_yaw,
                                         current_sample_time, &flat_state);

    mav_msgs::msgMultiDofJointTrajectoryFromEigen(flat_state, &msg);

    msg.points[0].time_from_start = ros::Duration(current_sample_time);
    // TODO(acmarkus) frame_id?
    command_publisher.publish(msg);

    current_sample_time += dt;
  } else {
    publish_timer.stop();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_sampling_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  command_publisher = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);

  ros::Subscriber trajectory_subscriber =
      nh.subscribe("path_segments", 10, trajectoryCallback);
  ros::ServiceServer stopt_srv =
      nh.advertiseService("stop_trajectory_sampling", stopCallback);
  pnh.param("dt", dt, 0.01);
  pnh.param("publish_whole_trajectory", publish_whole_trajectory, false);
  publish_timer =
      nh.createTimer(ros::Duration(dt), timerCallback, false, false);

  ros::spin();

  return EXIT_SUCCESS;
}
