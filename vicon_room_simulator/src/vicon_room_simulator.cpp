#include "vicon_room_simulator/vicon_room_simulator.hpp"

namespace vicon_room_simulator {

ViconRoom::ViconRoom(const ros::NodeHandle& nh) : nh_(nh), last_update_time_(ros::Time::now()) {
  ROS_INFO("Starting");
  gazebo_sub_ = nh_.subscribe("/gazebo/model_states", kQueueSize,
                                   &ViconRoom::gazeboCallback, this);
}

void ViconRoom::gazeboCallback(const gazebo_msgs::ModelStatesConstPtr& msg) {

  if((ros::Time::now() - last_update_time_).toSec() > (1.0/kViconFreq)){
    last_update_time_ = ros::Time::now();

    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (vicon_pubs_.find(msg->name[i]) == vicon_pubs_.end()) {
        ROS_INFO("Publishing state of model %s", msg->name[i].c_str());
        vicon_pubs_.emplace(msg->name[i],
                            nh_.advertise<geometry_msgs::TransformStamped>(
                                "ground_truth/" + msg->name[i], 0));
      }

      geometry_msgs::TransformStamped msg_out;
      msg_out.header.stamp = ros::Time::now();
      msg_out.header.frame_id = "world";
      msg_out.transform.translation.x = msg->pose[i].position.x;
      msg_out.transform.translation.y = msg->pose[i].position.y;
      msg_out.transform.translation.z = msg->pose[i].position.z;
      msg_out.transform.rotation = msg->pose[i].orientation;
      vicon_pubs_[msg->name[i]].publish(msg_out);
    }
  }
}
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "vicon_room_simulator");
  ros::NodeHandle nh;

  ROS_INFO("Ready to start");
  vicon_room_simulator::ViconRoom vicon_room(nh);

  ros::spin();

  return 0;
}
