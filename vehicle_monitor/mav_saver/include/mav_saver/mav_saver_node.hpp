/*
 * mav_saver_node.hpp
 *
 *  Created on: 18.06.2015
 *      Author: burrimi
 */

#ifndef INCLUDE_MAV_SAVER_MAV_SAVER_NODE_HPP_
#define INCLUDE_MAV_SAVER_MAV_SAVER_NODE_HPP_


#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/TransformStamped.h>
#include <mav_msgs/conversions.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Bool.h>

#include "mav_saver/mav_saver.hpp"

namespace mav_saver {

// Default values
static const std::string kDefaultTransformTopic = "transform";
static const std::string kDefaultTakeControlTopic = "take_control";


class MavSaverNode {
 public:
  MavSaverNode();

 private:
  ros::Subscriber transform_sub_;
  ros::Subscriber take_control_sub_;
  void TransformCallback(const geometry_msgs::TransformStampedConstPtr& msg);
  void TakeControlCallback(const std_msgs::BoolConstPtr& msg);

  std::shared_ptr<MavSaver> mav_saver_;
};

}


#endif /* INCLUDE_MAV_SAVER_MAV_SAVER_NODE_HPP_ */
