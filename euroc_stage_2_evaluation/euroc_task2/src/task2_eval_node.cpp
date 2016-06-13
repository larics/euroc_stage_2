
#include <euroc_stage2/task2_eval.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "task2_eval_node");

  ros::NodeHandle nh, private_nh("~");

  ROS_INFO("Started task2 eval node.");

  euroc_stage2::Task2Eval Task2Eval(nh, private_nh);

  ros::spin();

  return 0;
}
