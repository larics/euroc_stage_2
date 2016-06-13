/*
 * leica_marker.cpp
 *
 *  Created on: Jul 22, 2011
 *      Author: acmarkus
 */

#include <mav_viz/leica_marker.h>

namespace mav_viz {

LeicaMarker::LeicaMarker() {
  createLeica();
}

void LeicaMarker::createLeica() {
  visualization_msgs::Marker marker;
  const double tripod_height = 1.35;
  const double head_height = 0.3;

  // make rotors
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 1.35 / cos(15.0 / 180.0 * M_PI);
  marker.color.r = 1;
  marker.color.g = 1;
  marker.color.b = 0.0;
  marker.color.a = 1;
  marker.id = 0;

  marker.pose.position.z = -(tripod_height + head_height) / 2.0;

  // rotate by 15 deg around y: q= 0.9914 0.1305 0 0
  marker.pose.orientation.w = 0.9914;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = -0.1305;
  marker.pose.orientation.z = 0;

  marker.pose.position.x = 0.2;
  marker.pose.position.y = 0;
  markers_.push_back(marker);
  ++marker.id;
  markers_.push_back(marker);

  // rotate by 15 deg around y and 120 deg around z: q=0.4957   0.1130    -0.0653    0.8586
  marker.pose.orientation.w = 0.4957;
  marker.pose.orientation.x = 0.1130;
  marker.pose.orientation.y = -0.0653;
  marker.pose.orientation.z = 0.8586;

  marker.pose.position.x = -0.1;
  marker.pose.position.y = 0.17;
  markers_.push_back(marker);
  ++marker.id;
  markers_.push_back(marker);

  // rotate by 15 deg around y and -120 deg around z: q=-0.4957   0.1130   0.0653    0.8586
  marker.pose.orientation.w = -0.4957;
  marker.pose.orientation.x = 0.1130;
  marker.pose.orientation.y = 0.0653;
  marker.pose.orientation.z = 0.8586;

  marker.pose.position.x = -0.10;
  marker.pose.position.y = -0.17;
  ++marker.id;
  markers_.push_back(marker);

  // leica head
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = head_height;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1;
  marker.pose.orientation.w = 1;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  ++marker.id;
  markers_.push_back(marker);
}

}  // end namespace mav_viz
