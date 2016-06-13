#include <euroc_stage2/task3_eval.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "task3_eval_node");

  ros::NodeHandle nh, private_nh("~");

  ROS_INFO("Started task3 eval node.");

  euroc_stage2::Task3Eval Task3Eval(nh, private_nh);

  ros::spin();

  return 0;
}
