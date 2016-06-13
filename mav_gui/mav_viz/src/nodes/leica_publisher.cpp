/*

 Copyright (c) 2013, Jonas Londschien, ASL, ETH Zurich, Switzerland
 You can contact the author at <lojonas at ethz dot ch>

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

#include <mav_viz/leica_marker.h>

int main(int argc, char** argv)
{

  ros::init(argc, argv, "leica_publisher");
  ros::NodeHandle pnh("~");

  ros::Publisher marker_pub = pnh.advertise<visualization_msgs::MarkerArray>("marker_array", 10, true);

  std::string frame_id;
  double scale;

  pnh.param("frame_id", frame_id, std::string("leica"));
  pnh.param("scale", scale, 1.0);

  mav_viz::LeicaMarker leica;
  visualization_msgs::MarkerArray markers;

  leica.setLifetime(10.0);
  leica.setAction(visualization_msgs::Marker::ADD);

  std_msgs::Header header;
  header.frame_id = frame_id;

  while(ros::ok())
  {
    header.stamp = ros::Time::now();
    leica.setHeader(header);
    leica.getMarkers(markers, scale, false);
    marker_pub.publish(markers);
    ++header.seq;

    ros::Duration(5.0).sleep();
  }


}
