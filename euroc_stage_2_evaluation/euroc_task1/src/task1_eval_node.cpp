
#include <euroc_stage2/task1_eval.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task1_eval_node");

  ros::NodeHandle nh, private_nh("~");

  ROS_INFO("Started task1 eval node.");

  euroc_stage2::Task1Eval Task1Eval(nh, private_nh);

  ros::spin();

  return 0;
}
