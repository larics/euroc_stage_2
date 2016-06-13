/*

 Copyright (c) 2013, Markus Achtelik, ASL, ETH Zurich, Switzerland
 You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <planning_msgs/WayPointArray.h>
#include <planning_msgs/WaypointType.h>
#include <mav_planning_utils/polynomial.h>
#include <mav_msgs/CommandTrajectoryPositionYaw.h>

#include <list>

ros::Publisher ctrl_pub;
std::list<mav_msgs::CommandTrajectoryPositionYaw> mav_ctrls;
ros::Timer timer;
double dt;

template <int N>
bool fillMavWaypoints(
    const planning_msgs::WayPointArray::_waypoints_type& wpts) {
  if (wpts.empty()) {
    ROS_WARN("[polynomial sampling]: got 0 path segments");
    return false;
  }

  const size_t n_segments = wpts.size();

  typename mav_planning_utils::path_planning::Segment<N>::Vector sx(n_segments);
  typename mav_planning_utils::path_planning::Segment<N>::Vector sy(n_segments);
  typename mav_planning_utils::path_planning::Segment<N>::Vector sz(n_segments);
  typename mav_planning_utils::path_planning::Segment<N>::Vector syaw(
      n_segments);

  double path_time = 0;
  for (size_t s = 0; s < n_segments; ++s) {
    const planning_msgs::WayPoint& wps = wpts[s];

    bool ok = true;
    ok = ok && (wps.x.size() == N);
    ok = ok && (wps.y.size() == N);
    ok = ok && (wps.z.size() == N);
    ok = ok && (wps.yaw.size() == N);

    if (!ok) {
      ROS_WARN(
          "[polynomial sampling]: waypoints have different numbers of "
          "coefficients: %lu %lu %lu %lu",
          wps.x.size(), wps.y.size(), wps.z.size(), wps.yaw.size());
      return false;
    }

    sx[s].p.setCoefficients(
        Eigen::Map<const typename mav_planning_utils::Polynomial<N>::VectorR>(
            &wps.x[0]));
    sy[s].p.setCoefficients(
        Eigen::Map<const typename mav_planning_utils::Polynomial<N>::VectorR>(
            &wps.y[0]));
    sz[s].p.setCoefficients(
        Eigen::Map<const typename mav_planning_utils::Polynomial<N>::VectorR>(
            &wps.z[0]));
    syaw[s].p.setCoefficients(
        Eigen::Map<const typename mav_planning_utils::Polynomial<N>::VectorR>(
            &wps.yaw[0]));

    sx[s].t = sy[s].t = sz[s].t = syaw[s].t = wps.time;
    path_time += wps.time;
  }

  mav_ctrls.clear();

  mav_msgs::CommandTrajectoryPositionYaw ctrl;
  ctrl.header.frame_id = "/world";
  ctrl.header.stamp = ros::Time::now();

  for (double t = 0; t <= (path_time + 1e-12); t += dt) {
    ctrl.header.seq++;
    Eigen::Matrix<double, 4, 1> p;
    Eigen::Matrix<double, 2, 1> yaw_sample;

    mav_planning_utils::path_planning::samplePath(p, sx, t);
    ctrl.position.x = p[0];
    ctrl.velocity.x = p[1];
    ctrl.acceleration.x = p[2];
    ctrl.jerk.x = p[3];

    mav_planning_utils::path_planning::samplePath(p, sy, t);
    ctrl.position.y = p[0];
    ctrl.velocity.y = p[1];
    ctrl.acceleration.y = p[2];
    ctrl.jerk.y = p[3];

    mav_planning_utils::path_planning::samplePath(p, sz, t);
    ctrl.position.z = p[0];
    ctrl.velocity.z = p[1];
    ctrl.acceleration.z = p[2];
    ctrl.jerk.z = p[3];

    mav_planning_utils::path_planning::samplePath(yaw_sample, syaw, t);
    double yaw = fmod(yaw_sample[0], 2 * M_PI);
    yaw = yaw > M_PI ? yaw - 2 * M_PI : yaw;
    yaw = yaw < -M_PI ? yaw + 2 * M_PI : yaw;
    ctrl.yaw = yaw;
    ctrl.yaw_rate = yaw_sample[1];

    mav_ctrls.push_back(ctrl);
    ctrl.header.stamp += ros::Duration(dt);
  }

  return true;
}

void pathCb(const planning_msgs::WayPointArrayConstPtr& msg) {
  if (msg->waypoints.empty()) {
    ROS_WARN("[polynomial sampling]: received empty waypoint message");
    return;
  }

  // check if timer was allready running
  if (!mav_ctrls.empty()) {
    timer.stop();
    mav_ctrls.clear();
  }

  const planning_msgs::WayPoint::_type_type type = msg->waypoints[0].type;

  if (type == planning_msgs::WaypointType::POLYNOMIAL_12) {
    if (!fillMavWaypoints<12>(msg->waypoints)) return;
  } else if (type == planning_msgs::WaypointType::POLYNOMIAL_10) {
    if (!fillMavWaypoints<10>(msg->waypoints)) return;
  } else {
    ROS_WARN(
        "[polynomial sampling]: waypoints must have type POLYNOMIAL_12 or "
        "POLYNOMIAL_10, got %d instead ",
        type);
    return;
  }
  timer.start();
}

bool stopCb(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res) {
  timer.stop();
  mav_ctrls.clear();
  return true;
}

void timerCb(const ros::TimerEvent& e) {
  if (!mav_ctrls.empty()) {
    ctrl_pub.publish(mav_ctrls.front());
    mav_ctrls.pop_front();
  } else
    timer.stop();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "polynomial_sampling_node_new");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  nh.param("dt", dt, 0.01);

  ros::Subscriber sub = nh.subscribe("path_segments", 10, pathCb);
  ros::ServiceServer srv = nh.advertiseService("stop_polynomial_path", stopCb);
  timer = nh.createTimer(ros::Duration(dt), timerCb, false, false);
  ctrl_pub = nh.advertise<mav_msgs::CommandTrajectoryPositionYaw>(
      "fcu/control_new", 10);

  ros::spin();

  return 0;
}
