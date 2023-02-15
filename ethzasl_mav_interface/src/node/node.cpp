/*

Copyright (c) 2011-2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#include <aci/aci.h>
#include <aci/timesync_base.h>
#include <aci/uart.h>
#include <ros/ros.h>

#include <ethzasl_mav_interface/mav_interface.h>

using namespace ethzasl_mav_interface;

void printTopicInfo()
{
  //  print published/subscribed topics
  std::string node_name = ros::this_node::getName();
  ros::V_string topics;
  ros::this_node::getSubscribedTopics(topics);
  std::string topics_string = node_name + ":\n\tsubscribed to topics:\n";
  for (unsigned int i = 0; i < topics.size(); i++)
    topics_string += ("\t\t" + topics.at(i) + "\n");

  topics_string += "\tadvertised topics:\n";
  ros::this_node::getAdvertisedTopics(topics);
  for (unsigned int i = 0; i < topics.size(); i++)
    topics_string += ("\t\t" + topics.at(i) + "\n");

  ROS_INFO_STREAM(""<< topics_string);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "fcu");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  MavInterface mav_interface;
  bool success = mav_interface.start(nh, private_nh);
  if (!success) {
    ROS_FATAL("unable to start mav interface");
    return EXIT_FAILURE;
  }

  ros::spin();

  return 0;
}
