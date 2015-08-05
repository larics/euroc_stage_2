/*
 * mav_saver_node.cpp
 *
 *  Created on: 18.06.2015
 *      Author: burrimi
 */

#include "mav_saver/mav_saver_node.hpp"


namespace mav_saver {

MavSaverNode::MavSaverNode() {

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  mav_saver_.reset(new MavSaver(nh, pnh));

  transform_sub_ = nh.subscribe(kDefaultTransformTopic, 10,
                                &MavSaverNode::TransformCallback, this);

  take_control_sub_ = nh.subscribe(kDefaultTakeControlTopic, 10,
                                &MavSaverNode::TakeControlCallback, this);
}

void MavSaverNode::TransformCallback(const geometry_msgs::TransformStampedConstPtr& msg) {
  Eigen::Vector3d p_W_I = mav_msgs::vector3FromMsg(msg->transform.translation);
  Eigen::Quaterniond q_W_I = mav_msgs::quaternionFromMsg(msg->transform.rotation);

  mav_saver_->SetPose(p_W_I, q_W_I);
}

void MavSaverNode::TakeControlCallback(const std_msgs::BoolConstPtr& msg) {
  mav_saver_->SetTakeControlFlag(msg->data);
}


}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mav_saver_node");

  mav_saver::MavSaverNode mav_saver_node;

  ros::spin();

  return 0;
}
