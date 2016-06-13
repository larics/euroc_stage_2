
#include <euroc_stage2/task1_server.h>

namespace euroc_stage2 {

Task1Server::Task1Server(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
{
  start_publishing_service_ = private_nh.advertiseService(
      "start_publishing", &Task1Server::StartPublishing, this);

  idle_mode_=true;
}

Task1Server::~Task1Server()
{
}


bool Task1Server::StartPublishing(std_srvs::Empty::Request& request,
                                         std_srvs::Empty::Response& response) {

  ROS_INFO_STREAM("task1 starts publishing.");
  idle_mode_=false;

  return true;
}

};
