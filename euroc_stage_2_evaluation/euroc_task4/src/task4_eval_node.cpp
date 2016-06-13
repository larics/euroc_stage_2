
#include <euroc_stage2/task4_eval.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "task4_eval_node");

  ros::NodeHandle nh, private_nh("~");

  ROS_INFO("Started task4 eval node.");

  euroc_stage2::Task4Eval task4_eval(nh, private_nh);

  ros::spin();

  return 0;
}
