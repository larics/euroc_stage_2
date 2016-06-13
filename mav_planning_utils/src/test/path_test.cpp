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

#include <mav_planning_utils/path_planning.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <mav_planning_utils/conversions.h>
#include <mav_planning_utils/eigen_visualization.h>

using namespace mav_planning_utils;
using namespace mav_planning_utils::path_planning;

using namespace visualization_msgs;

void insertMarkers(MarkerArray& m, MarkerArray to_insert) {
  m.markers.insert(m.markers.end(), to_insert.markers.begin(),
                   to_insert.markers.end());
}

int main(int argc, char** argv) {
  Path4D<12> path;
  std::vector<Vertex4D> sv;
  Eigen::Matrix<double, 4, 1> tmp;

  sv.push_back(Vertex4D(10, 2, 4));

  sv.push_back(Vertex4D(10, 2));
  sv.back().addConstraint(0, (tmp << 5, 0, 2, 0).finished());

  sv.push_back(Vertex4D(2, 2));
  sv.back().addConstraint(0, (tmp << 5, 5, 3, 0).finished());
  sv.back().addConstraint(DerivativesP::v, (tmp << 0, 0, 0, 0).finished());

  sv.push_back(Vertex4D(2, 2));
  sv.back().addConstraint(0, (tmp << 0, 5, 2, 0).finished());

  sv.push_back(Vertex4D(10, 2, 4));
  sv.back().addConstraint(0, (tmp << -5, 5, 2, 0).finished());

  Vertex1D cp(1, 1);
  cp.addConstraint(1, 3);

  path.optimizeWithTime(sv, 4, cp, cp);

  Motion4D<5, 2>::Vector data;
  path.sample<5, 2>(data, 0.1);

  for (size_t i = 0; i < data.size(); i += 5) {
    std::cout << data[i].toString() << std::endl;
  }
  std::cout << data.back().toString() << std::endl;

  ros::init(argc, argv, "path_test");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<MarkerArray>("path", 1, true);

  MarkerArray markers, markers_tmp;
  std_msgs::Header header;
  header.frame_id = "/world";
  header.stamp = ros::Time::now();
  header.seq = 0;

  MavState s;

  double magnitude_of_gravity = 9.81;

  motion4dToMulticopter(s, data.front(), magnitude_of_gravity);
  eigen_visualization::drawAxesArrows(markers_tmp, s.p, s.q, 0.6);
  insertMarkers(markers, markers_tmp);

  for (size_t i = 0; i < data.size(); i += 5) {
    motion4dToMulticopter(s, data[i], magnitude_of_gravity);
    eigen_visualization::drawAxesArrows(markers_tmp, s.p, s.q, 0.3);
    insertMarkers(markers, markers_tmp);

    Marker marker;
    eigen_visualization::drawArrow(
        marker, s.p, s.p + s.a,
        eigen_visualization::createColorRGBA(1, 1, 0, 1));
    markers.markers.push_back(marker);

    eigen_visualization::drawArrow(
        marker, s.p, s.p + s.v,
        eigen_visualization::createColorRGBA(0, 1, 1, 1));
    markers.markers.push_back(marker);
  }

  motion4dToMulticopter(s, data.back(), magnitude_of_gravity);
  eigen_visualization::drawAxesArrows(markers_tmp, s.p, s.q, 0.6);
  insertMarkers(markers, markers_tmp);

  // write header etc
  int cnt = 0;
  for (MarkerArray::_markers_type::iterator it = markers.markers.begin();
       it != markers.markers.end(); ++it) {
    it->action = Marker::ADD;
    it->header = header;
    it->id = cnt;
    it->ns = "path";
    cnt++;
  }

  pub.publish(markers);

  while (ros::ok()) usleep(1e5);
}
