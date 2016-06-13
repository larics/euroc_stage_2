#ifndef TASK1_SERVER_H
#define TASK1_SERVER_H

#include <Eigen/Eigen>
#include <stdio.h>
#include <fstream>
#include <iostream>

//ros
#include <ros/ros.h>
#include <ros/callback_queue.h>

//ros msgs
#include <mav_msgs/default_topics.h>
#include <std_srvs/Empty.h>


namespace euroc_stage2 {

class Task1Server
{
 public:
  Task1Server(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  ~Task1Server();

 private:
   bool StartPublishing(std_srvs::Empty::Request& request,
                                           std_srvs::Empty::Response& response);

  //subscribers

  //publishers


  ros::ServiceServer start_publishing_service_;

  bool idle_mode_;
};

}
#endif
