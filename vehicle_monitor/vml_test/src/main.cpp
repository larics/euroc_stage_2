#include "vehicle_monitor_library/VehicleMonitor.hpp"
#include "vehicle_monitor_library/VehicleMonitorObserver.hpp"
#include "vehicle_monitor_library/Vehicle.hpp"
#include "vehicle_monitor_library/SimpleVelocityEstimator.hpp"
#include "vehicle_monitor_library/CollisionConstraintChecker.hpp"
#include "vehicle_monitor_library/OutOfSpaceConstraintChecker.hpp"
#include "vehicle_monitor_library/MotionCaptureSystemFrame.hpp"

#include <random>
#include <chrono>
#include <thread>
#include <string>

#include <iostream>

#include <octomap/octomap_types.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include "yaml-cpp/yaml.h"

#include <boost/filesystem.hpp>

#include "ros/ros.h"
#include "ros/package.h"
#include "ros/publisher.h"
#include "geometry_msgs/PointStamped.h"

using namespace std;
using namespace VehicleMonitorLibrary;




class TestVehicleMonitorObserver : public VehicleMonitorObserverBase {


  virtual void Update(const std::map<std::string, std::map<std::string, ConstraintCheckerOutput> >& vehicleStatus){

    for(const auto & vehicleStatusMapElement : vehicleStatus){

      for(const auto & outputMapElement : vehicleStatusMapElement.second){

        if(outputMapElement.second._constraintSatisfied == false){
          cout << "[" << vehicleStatusMapElement.first << "] error: " <<
              outputMapElement.first << " - Last Valid Position: " <<
              outputMapElement.second._lastValidState._linear << endl;
        }

      }

    }

  }


};


int main(int argc, char **argv){


  ros::init(argc, argv, "vml_test");
  ros::NodeHandle n;

  string configFilePath = ros::package::getPath("vml_test");

  stringstream ss;
  ss << configFilePath << "/res/config.yaml";

  YAML::Node config = YAML::LoadFile(ss.str());

  ss.str("");
  ss << configFilePath << "/res/FMANoWalls.bt";

  boost::filesystem::path octoMapPath(ss.str());

  YAML::Node nodeCorner1 = config["bounding_box_vertexA"];
  Eigen::Vector3d vertexA(nodeCorner1[0].as<float>(), nodeCorner1[1].as<float>(), nodeCorner1[2].as<float>());

  YAML::Node nodeCornerB = config["bounding_box_vertexB"];
  Eigen::Vector3d vertexB(nodeCornerB[0].as<float>(), nodeCornerB[1].as<float>(), nodeCornerB[2].as<float>());

  unsigned int motionCaptureSystemFrequency = config["motion_capture_frequency"].as<unsigned int>();

  float collisionThreesholdInBoundingSphereRadius = config["collision_threeshold_in_sphere_radius"].as<float>();
  float maxDistToCheckCollision = config["max_distance_to_check_collision"].as<float>();

  unsigned int projectionWindow = config["projection_window"].as<unsigned int>();

  VehicleMonitor vehicleMonitor(octoMapPath, vertexA, vertexB, motionCaptureSystemFrequency);

  vehicleMonitor.RegisterObserver(std::make_shared<TestVehicleMonitorObserver>());

  std::shared_ptr<SimpleVelocityEstimator> velocityEstimator =
      std::make_shared<SimpleVelocityEstimator>(motionCaptureSystemFrequency);

  std::shared_ptr<CollisionConstraintChecker> collisionChecker =
      std::make_shared<CollisionConstraintChecker>(vehicleMonitor.GetOcTreePtr(),
                                                   vehicleMonitor.GetEnvironmentBoundingVolume(),
                                                   maxDistToCheckCollision, // max distance to check for collisions
                                                   collisionThreesholdInBoundingSphereRadius, // we consider a collision when the distance is <= 2*radius
                                                   projectionWindow,
                                                   motionCaptureSystemFrequency,
                                                   velocityEstimator
      );

  vehicleMonitor.RegisterChecker(collisionChecker);

  std::shared_ptr<OutOfSpaceConstraintChecker> outOfSpaceChecker =
      std::make_shared<OutOfSpaceConstraintChecker>(vehicleMonitor.GetEnvironmentBoundingVolume());

  vehicleMonitor.RegisterChecker(outOfSpaceChecker);

  ss.str("");
  ss << configFilePath << "/res/vehicles.yaml";

  YAML::Node vehicleRootNode = YAML::LoadFile(ss.str());

  YAML::Node vehicleNode;
  for(unsigned int i = 0; i < vehicleRootNode["Vehicles"].size(); i++){

    vehicleNode = vehicleRootNode["Vehicles"][i];
    std::shared_ptr<Vehicle> vehiclePtr = std::make_shared<Vehicle>(vehicleNode["ID"].as<string>(), vehicleNode["Radius"].as<float>());
    cout << *vehiclePtr;
    vehicleMonitor.RegisterVehicle(vehiclePtr);

  }

  std::mt19937 generator( static_cast<unsigned int>( std::chrono::high_resolution_clock::now().time_since_epoch().count()) );
  std::uniform_real_distribution<float> linearVelDistribution( -0.5f, 0.5f );
  std::uniform_real_distribution<float> angularDistribution( -0.1f, 0.1f );

  map<string, VehicleState> lastState;
  map<string, ros::Publisher> posePublishers;
  for(auto vehicleID : vehicleMonitor.GetVehicleIDs()){
    lastState[vehicleID] = VehicleState();
    stringstream ss;
    ss << "PoseVehicle" << vehicleID;
    posePublishers.insert(make_pair(vehicleID, n.advertise<geometry_msgs::PointStamped>(ss.str(), 100)));
  }

  ros::Publisher octomapPublisher = n.advertise<octomap_msgs::Octomap>("vml_octomap", 1);
  octomap_msgs::Octomap octomapMsg;
  octomapMsg.header.stamp = ros::Time::now();
  octomapMsg.header.frame_id = "map";

  octomap_msgs::binaryMapToMsg(*vehicleMonitor.GetOcTreePtr(), octomapMsg);


  octomapPublisher.publish(octomapMsg);

  MotionCaptureSystemFrame frame(0);

  geometry_msgs::PointStamped pointOutput;
  pointOutput.header.frame_id = "map";

  cout << "--------- " << __cplusplus << endl;

  ros::Rate loop_rate(200);
  int i = 0;

  while(ros::ok()) {

    bool emergency = false;

    if(i%50 == 1){
      emergency = true;
    }

    // publishing it once in a while because rviz not always show
    // the octomap if published only at the beginning
    if(i%1000 == 0){
      octomapPublisher.publish(octomapMsg);
    }

    frame.SetFrameNumber(i);

    ros::Time now = ros::Time::now();

    pointOutput.header.stamp = now;

    for(auto vehicleID : vehicleMonitor.GetVehicleIDs()){

      Eigen::Vector3d & position = lastState[vehicleID]._linear;

      int id = boost::lexical_cast<float>(vehicleID);

      pointOutput.point.x = position.x() = 2.0 * cos(M_PI/4*(now.sec+now.nsec/1e9));
      pointOutput.point.y = position.y() = 2.0 * sin(M_PI/4*(now.sec+now.nsec/1e9));
      pointOutput.point.z = position.z() = id+1;

      Eigen::Vector3d & orientation = lastState[vehicleID]._angular;

      orientation.x() += angularDistribution(generator);
      orientation.y() += angularDistribution(generator);
      orientation.z() += angularDistribution(generator);

      posePublishers[vehicleID].publish(pointOutput);

      // Uncomment below to publish projected pose of vehicle 1

      //			if(vehicleID == "1"){
      //				posePublishers[vehicleID].publish(pointOutput);
      //
      //				Eigen::Vector3d estimatedLinearVelocity;
      //				Eigen::Vector3d estimatedAngularVelocity;
      //
      //				velocityEstimator->PredictVelocity(vehicleID,
      //									estimatedLinearVelocity, estimatedAngularVelocity);
      //
      //				pointOutput.point.x =
      //						pointOutput.point.x + estimatedLinearVelocity.x() * projectionWindow / motionCaptureSystemFrequency;
      //				pointOutput.point.y =
      //						pointOutput.point.y + estimatedLinearVelocity.y() * projectionWindow / motionCaptureSystemFrequency;
      //				pointOutput.point.z =
      //						pointOutput.point.z + estimatedLinearVelocity.z() * projectionWindow / motionCaptureSystemFrequency;
      //
      //				posePublishers["2"].publish(pointOutput);
      //
      //			}

      frame.UpdateFrameElement(vehicleID, lastState[vehicleID]);

    }

    vehicleMonitor.Trigger(frame, emergency);

    ++i;

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;

}
