/*
 * marker_group.h
 *
 *  Created on: Jul 22, 2011
 *      Author: acmarkus
 */

#ifndef MARKER_GROUP_H_
#define MARKER_GROUP_H_

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace mav_viz {

typedef std::vector<visualization_msgs::Marker> MarkerVector;

class MarkerGroup {

 protected:
  std::string name_;
  std::string description_;
  MarkerVector markers_;
  static void transformMarker(visualization_msgs::Marker & marker, const Eigen::Vector3d & t,
                              const Eigen::Quaterniond & q);
 public:
  MarkerGroup();
  virtual ~MarkerGroup();
  void getMarkers(visualization_msgs::MarkerArray & marker_array, const double & scale = 1, bool append =
                      false) const;
  void getMarkers(MarkerVector & markers, const double & scale = 1, bool append = false) const;
  void setNamespace(const std::string & ns);
  void setHeader(const std_msgs::Header & header);
  void setHeaderAndNamespace(const std_msgs::Header & header, const std::string & ns);
  void setAction(const int32_t & action);
  void setLifetime(double lifetime);
  void setFrameLocked(bool locked);
  void transform(const Eigen::Vector3d & t, const Eigen::Quaterniond & q);
  void publish(ros::Publisher & pub);
};

}  // end namespace mav_viz
#endif /* MARKER_GROUP_H_ */
