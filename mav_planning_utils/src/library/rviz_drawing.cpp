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

#include <mav_planning_utils/rviz_drawing.h>

using namespace visualization_msgs;

namespace mav_planning_utils {

RvizDrawing::RvizDrawing(const std::string& name, const std::string& base_name,
                         const std::string& frame_id)
    : marker_lifetime_(10.0),
      nh_(base_name),
      name_(name),
      marker_id_(0),
      spinner_(0) {
  header_.frame_id = frame_id;
  pub_ = nh_.advertise<visualization_msgs::MarkerArray>(name_, 10, true);
  spinner_.start();
}

RvizDrawing::~RvizDrawing() {
  clear();
  spinner_.stop();
}

void RvizDrawing::clear() {
  pub_timer_.stop();
  MarkerArrayPtr msg(new MarkerArray);
  header_.stamp = ros::Time::now();
  msg->markers.resize(to_clear_.size());
  int i = 0;
  for (MarkerClearSet::const_iterator it = to_clear_.begin();
       it != to_clear_.end(); ++it) {
    Marker& m = msg->markers[i];
    m.id = it->first;
    m.ns = it->second;
    m.header = header_;
    m.action = Marker::DELETE;
    ++i;
  }

  pub_.publish(msg);
  to_clear_.clear();
  to_draw_.clear();
  //  ++header_.seq;
  header_.seq = 0;
}

void RvizDrawing::draw() {
  pub_timer_ = nh_.createTimer(marker_lifetime_, &RvizDrawing::draw, this);
  draw(ros::TimerEvent());
}

void RvizDrawing::draw(const ros::TimerEvent& e) {
  MarkerArrayPtr msg(new MarkerArray);
  header_.stamp = ros::Time::now();
  msg->markers.resize(to_draw_.size());

  //  std::cout<<"size to draw: "<<to_draw_.size()<<std::endl;
  int i = 0;
  for (MarkerMap::const_iterator it = to_draw_.begin(); it != to_draw_.end();
       it++) {
    Marker& m = msg->markers[i];
    m = it->second;
    m.header = header_;
    m.lifetime = marker_lifetime_;
    ++i;
  }

  pub_.publish(msg);
  //  to_draw_.clear();
  ++header_.seq;
}

int RvizDrawing::addModifyMarker(const visualization_msgs::Marker& marker,
                                 const std::string& ns, int id) {
  std::pair<MarkerMap::iterator, bool> ret;
  Marker::_action_type action = Marker::MODIFY;
  if (id == -1) {
    action = Marker::ADD;
    id = marker_id_;
    ++marker_id_;
  }

  ret = to_draw_.insert(MarkerMap::value_type(id, marker));
  Marker& m = ret.first->second;
  // overwrite if element existed
  if (!ret.second) {
    m = marker;
  }
  if (!ns.empty())
    m.ns = name_ + "/" + ns;
  else
    m.ns = name_;
  m.id = id;
  m.action = action;

  to_clear_.insert(
      std::make_pair<int, std::string>(int(m.id), std::string(m.ns)));

  return id;
}

int RvizDrawing::addModifyMarkerArray(
    const visualization_msgs::MarkerArray& marker_array, const std::string& ns,
    int id) {
  const visualization_msgs::MarkerArray::_markers_type& markers =
      marker_array.markers;

  if (markers.size() < 1) return -1;

  int ret = addModifyMarker(markers[0], ns, id);
  for (size_t i = 1; i < markers.size(); ++i) {
    if (id == -1)
      addModifyMarker(markers[i], ns, id);
    else
      addModifyMarker(markers[i], ns, id + i);
  }
  return ret;
}

// void RvizDrawing::wipe(int n)
//{
//  MarkerArrayPtr msg(new MarkerArray);
//  Marker marker;
//  marker.action = Marker::DELETE;
//  marker.ns = name_;
//  marker.header = header_;
//  msg->markers.resize(n, marker);
//
//  for (int i = 0; i < n; i++)
//  {
//    msg->markers[i].id = i;
//  }
//
//  pub_.publish(msg);
//  to_clear_.clear();
//  to_draw_.clear();
//  marker_id_ = 0;
//  usleep(1e5);
//}

} /* namespace rrbt */
