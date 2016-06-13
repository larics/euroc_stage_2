#ifndef INCLUDE_GROUND_TRUTH_OCTREE_HPP_
#define INCLUDE_GROUND_TRUTH_OCTREE_HPP_

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <map>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include "ros/package.h"

#include <octomap_world/octomap_manager.h>
#include <octomap_world/octomap_world.h>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf_conversions/tf_eigen.h>

#include <boost/filesystem.hpp>

#define QUEUE_SIZE 1000

namespace ground_truth_octree {

// Default values
const std::string kDefaultRoomOctree = "res/LeoC6.bt";
// pairs of vicon tracker topics and octomaps of the same object
const std::map<std::string, std::string> kDefaultObstacles = {
    {"ground_truth/scaffold_a", "res/ScaffoldA.bt"},
    {"ground_truth/scaffold_b", "res/ScaffoldB.bt"}};
constexpr double kDefaultMaxUpdateRate = 1;
constexpr bool kAlwaysUpdate = false;
constexpr int kNumOfUpdatesAtStart = 5;

// Holds objects that are defined by an octomap and a vicon tracker topic. The
// octomaps are internally converted to point clouds to allow them to be
// transformed, resampled and combined with other object
class ObjectPoints {
 public:
  typedef std::shared_ptr<ObjectPoints> Ptr;

  ObjectPoints(const std::string& octree_path, double octree_res);

  ObjectPoints(const ros::NodeHandle& nh, const std::string& key,
               const std::string& octree_path, double octree_res);

  void addPointsToOctree(octomap::OcTree* octree);

  void viconCallback(const geometry_msgs::TransformStampedConstPtr& msg);

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> getTformedPoints(void) const;

 private:

  ros::NodeHandle nh_;
  pcl::PointCloud<pcl::PointXYZ> point_cloud_;

  tf::Transform pose_;

  ros::Subscriber vicon_sub_;

  void loadPoints(const std::string& octree_path, double octree_res);


std::string processPath(const std::string& octomap_path_string); 
};

// Combines multiple ObjectPoint objects with a octomap of a room to form a
// contniuously updated octomap of a vicon room containing several tracked
// objects.
class UpdatingOctree : public volumetric_mapping::OctomapManager {
 public:
  typedef std::shared_ptr<UpdatingOctree> Ptr;

  UpdatingOctree(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

  bool updateService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  void viconCallback(const geometry_msgs::TransformStampedConstPtr& msg);

 private:

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Time previous_send_time_;
  double max_update_rate_;
  bool always_update_;
  int num_of_updates_at_start_;
  int current_update_num_;

  ros::Subscriber vicon_sub_;
  ros::ServiceServer update_service_; 

  std::vector<ObjectPoints::Ptr> objects_;

  ObjectPoints::Ptr room_;

  void regenerateOctree(void);

  void addObjectToOctree(const ObjectPoints& object);
};
}

#endif /* INCLUDE_GROUND_TRUTH_OCTREE_HPP_ */
