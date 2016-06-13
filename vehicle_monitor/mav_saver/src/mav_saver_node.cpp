/*
 * mav_saver_node.cpp
 *
 *  Created on: 18.06.2015
 *      Author: burrimi
 */

#include "mav_saver/mav_saver_node.hpp"

namespace mav_saver {

MavSaverNode::MavSaverNode() : valid_odometry_(false) {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  mav_saver_.reset(new MavSaver(nh, pnh));

  octomap_sub_ = nh.subscribe(kDefaultOctomapTopic, 1,
                                &MavSaverNode::OctomapCallback, this);

  transform_sub_ = nh.subscribe(kDefaultTransformTopic, 10,
                                &MavSaverNode::TransformCallback, this);

  odometry_sub_ = nh.subscribe(kDefaultOdometryTopic, 10,
                               &MavSaverNode::OdometryCallback, this);

  take_control_sub_ = nh.subscribe(kDefaultTakeControlTopic, 10,
                                   &MavSaverNode::TakeControlCallback, this);
}

void MavSaverNode::TransformCallback(
    const geometry_msgs::TransformStampedConstPtr& msg) {
  if (valid_odometry_) {
    ROS_WARN_THROTTLE(0.5,
                      "Got transform message AND odometry. Please only "
                      "subscribe to either transform OR odometry");
    return;
  }

  Eigen::Vector3d p_W_I = mav_msgs::vector3FromMsg(msg->transform.translation);
  Eigen::Quaterniond q_W_I =
      mav_msgs::quaternionFromMsg(msg->transform.rotation);

  mav_saver_->setPose(p_W_I, q_W_I);
}

void MavSaverNode::OctomapCallback(const octomap_msgs::OctomapConstPtr& msg) {
  mav_saver_->setOctomap(msg);
}

void MavSaverNode::OdometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  valid_odometry_ = true;

  mav_msgs::EigenOdometry odometry;
  mav_msgs::eigenOdometryFromMsg(*msg, &odometry);

  mav_saver_->setOdometry(odometry.position_W, odometry.orientation_W_B,
                          odometry.getVelocityWorld(),
                          odometry.angular_velocity_B);
}

void MavSaverNode::TakeControlCallback(const std_msgs::BoolConstPtr& msg) {
  if (msg->data && !mav_saver_->getTakeControlFlag()) {
    mav_saver_->setTakeControlFlag(true);
    ROS_WARN_THROTTLE(
        1, "[MAV_SAVER]: EXTERNAL MAV RESCUE REQUESTED, TAKING OVER !!!");
  }
}
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mav_saver_node");

  mav_saver::MavSaverNode mav_saver_node;

  ros::spin();

  return 0;
}
