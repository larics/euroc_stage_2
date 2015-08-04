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
#include <nodelet/nodelet.h>

#include <ethzasl_mav_interface/mav_interface.h>

namespace ethzasl_mav_interface {

class MavInterfaceNodelet : public nodelet::Nodelet {
  virtual void onInit();
};

void MavInterfaceNodelet::onInit() {
  ros::NodeHandle nh = nodelet::Nodelet::getNodeHandle();
  ros::NodeHandle private_nh = nodelet::Nodelet::getPrivateNodeHandle();
  MavInterface mav_interface;
  bool success = mav_interface.start(nh, private_nh);

  // There is unfortunately no nice way to shut down the nodelet.
  // Thus, we throw this exception and let the nodelet manager handle it.
  if (!success) {
    ROS_FATAL("Unable to load nodelet");
    throw nodelet::Exception("Unable to load nodelet");
  }
}

} // end namespace ethzasl_mav_interface

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ethzasl_mav_interface::MavInterfaceNodelet, nodelet::Nodelet)
