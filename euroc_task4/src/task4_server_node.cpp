
#include <euroc_stage2/task4_server.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "task4_server_node");

  ros::NodeHandle nh, private_nh("~");

  ROS_INFO("Started task4 server node.");

  euroc_stage2::Task4Server task4_server(nh, private_nh);

  ros::spin();

  return 0;
}
