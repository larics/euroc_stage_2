#include "ground_truth_octree/ground_truth_octree.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_truth_octree");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ground_truth_octree::UpdatingOctree updating_octree(nh, private_nh);

  ros::spin();

  return 0;
}