
#include <euroc_stage2/task5_server.h>

namespace euroc_stage2 {

Task5Server::Task5Server(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
{
  start_publishing_service_ = private_nh.advertiseService(
      "start_publishing", &Task5Server::StartPublishing, this);

  idle_mode_=true;
}

Task5Server::~Task5Server()
{
}


bool Task5Server::StartPublishing(std_srvs::Empty::Request& request,
                                         std_srvs::Empty::Response& response) {

  ROS_INFO_STREAM("task5 starts publishing.");
  idle_mode_=false;

  return true;
}

};
