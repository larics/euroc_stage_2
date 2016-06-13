
#include <euroc_stage2/task3_server.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "task3_server_node");

  ros::NodeHandle nh, private_nh("~");

  ROS_INFO("Started task3 server node.");

  euroc_stage2::Task3Server Task3Server(nh, private_nh);

  ros::spin();

  return 0;
}
