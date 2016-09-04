#include "ground_truth_octree/ground_truth_octree.hpp"

namespace ground_truth_octree {

UpdatingOctree::UpdatingOctree(const ros::NodeHandle& nh,
                               const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      volumetric_mapping::OctomapManager(nh, private_nh),
      previous_send_time_(ros::Time::now()),
      current_update_num_(0) {
  std::map<std::string, std::string> obstacles_info;
  std::string room_octree_path;
  private_nh_.param("Obstacles_info", obstacles_info, kDefaultObstacles);
  private_nh_.param("Room_octree", room_octree_path, kDefaultRoomOctree);
  private_nh_.param("max_update_rate", max_update_rate_, kDefaultMaxUpdateRate);
  private_nh_.param("always_update", always_update_, kAlwaysUpdate);
  private_nh_.param("num_of_updates_at_start", num_of_updates_at_start_,
                    kNumOfUpdatesAtStart);

  room_ = std::make_shared<ObjectPoints>(room_octree_path, getResolution());

  update_timer_ = nh_.createTimer(ros::Duration(max_update_rate_), &UpdatingOctree::viconCallback, this);

  for (std::pair<const std::string, std::string>& info : obstacles_info) {
    ROS_INFO("subscribed to %s", info.first.c_str());

    objects_.emplace_back(std::make_shared<ObjectPoints>(
        nh_, info.first, info.second, getResolution()));
  }

  update_service_ = private_nh_.advertiseService("update_now", &UpdatingOctree::updateService, this);
}

void UpdatingOctree::regenerateOctree(void) {
  // Resets to a brand now octree (octree_->clear() and by extension resetMap()
  // both result in octree_->setNodeValue crashing for some reason)
  octree_.reset(new octomap::OcTree(params_.resolution));
  addObjectToOctree(*room_);
  for (ObjectPoints::Ptr& object : objects_) {
    addObjectToOctree(*object);
  }
}

void UpdatingOctree::addObjectToOctree(const ObjectPoints& object) {
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> points =
      object.getTformedPoints();
  // set points manually, octomap_manager interface is too slow
  for (pcl::PointXYZ& point : *points) {
    octree_->setNodeValue(octomath::Vector3(point.x, point.y, point.z),
                          octree_->getClampingThresMaxLog(), true);
  }

  octree_->updateInnerOccupancy();
}

bool UpdatingOctree::updateService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  regenerateOctree();
  publishAll();
  return true;
}

void UpdatingOctree::viconCallback(const ros::TimerEvent& e) {
  if (always_update_ || (num_of_updates_at_start_ > current_update_num_)) {
    regenerateOctree();
    publishAll();
    ++current_update_num_;
  }
}

ObjectPoints::ObjectPoints(const std::string& octree_path, double octree_res) {
  loadPoints(octree_path, octree_res);
  has_pose_ = true;
}

ObjectPoints::ObjectPoints(const ros::NodeHandle& nh, const std::string& topic,
                           const std::string& octree_path, double octree_res)
    : nh_(nh) {
  vicon_sub_ =
      nh_.subscribe(topic, QUEUE_SIZE, &ObjectPoints::viconCallback, this);

  has_pose_ = false;
  loadPoints(octree_path, octree_res);
}

void ObjectPoints::loadPoints(const std::string& octree_path,
                              double octree_res) {
  pose_.setIdentity();
  volumetric_mapping::OctomapWorld object_octree;
  object_octree.loadOctomapFromFile(processPath(octree_path));

  pcl::PointCloud<pcl::PointXYZ> raw_cloud;
  object_octree.getOccupiedPointcloudInBoundingBox(
      object_octree.getMapCenter(), object_octree.getMapSize(), &raw_cloud);

  // upsample cloud
  size_t num_samples =
      std::ceil(static_cast<float>(object_octree.getResolution()) / octree_res);
  float gap_between_samples =
      static_cast<float>(object_octree.getResolution()) /
      static_cast<float>(num_samples);
  float offset = object_octree.getResolution() / 2;

  for (pcl::PointXYZ& it : raw_cloud) {
    for (float x = gap_between_samples / 2 - offset; x <= offset;
         x += gap_between_samples) {
      for (float y = gap_between_samples / 2 - offset; y <= offset;
           y += gap_between_samples) {
        for (float z = gap_between_samples / 2 - offset; z <= offset;
             z += gap_between_samples) {
          pcl::PointXYZ point(x + it.x, y + it.y, z + it.z);
          point_cloud_.push_back(point);
        }
      }
    }
  }
  ROS_INFO("Loaded %lu points", point_cloud_.size());
}

std::string ObjectPoints::processPath(const std::string& octomap_path_string) {
  std::string configFilePath = ros::package::getPath("ground_truth_octree");
  std::stringstream ss;
  ss.str("");

  if (octomap_path_string[0] == '/') {
    ss << octomap_path_string;
  } else {
    ss << configFilePath << '/' << octomap_path_string;
  }

  boost::filesystem::path octomap_path(ss.str());

  if (boost::filesystem::exists(octomap_path) &&
      boost::filesystem::is_regular_file(octomap_path)) {
    return octomap_path.string();
  } else {
    ROS_ERROR("Invalid path to octree %s", ss.str().c_str());
  }

  return "";
}

void ObjectPoints::viconCallback(
    const geometry_msgs::TransformStampedConstPtr& msg) {
  tf::transformMsgToTF(msg->transform, pose_);
  has_pose_ = true;
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> ObjectPoints::getTformedPoints(
    void) const {
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> tformed_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  if(has_pose_){
    Eigen::Affine3d temp_eigen_tf;
    tf::transformTFToEigen(pose_, temp_eigen_tf);
    pcl::transformPointCloud(point_cloud_, *tformed_cloud, temp_eigen_tf);
  }
  return tformed_cloud;
}
};