
#include <euroc_stage2/task5_server.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task5_server_node");

  ros::NodeHandle nh, private_nh("~");

  ROS_INFO("Started task5 server node.");

  euroc_stage2::Task5Server Task5Server(nh, private_nh);

  ros::spin();

  return 0;
}
