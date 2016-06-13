
#include <euroc_stage2/task1_server.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task1_server_node");

  ros::NodeHandle nh, private_nh("~");

  ROS_INFO("Started task1 server node.");

  euroc_stage2::Task1Server Task1Server(nh, private_nh);

  ros::spin();

  return 0;
}
