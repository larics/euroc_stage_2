
#include <euroc_stage2/task2_server.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "task2_server_node");

  ros::NodeHandle nh, private_nh("~");

  ROS_INFO("Started task2 server node.");

  euroc_stage2::Task2Server Task2Server(nh, private_nh);

  ros::spin();

  return 0;
}
