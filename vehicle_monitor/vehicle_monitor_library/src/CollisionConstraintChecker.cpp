/**
 *    @file CollisionConstraintChecker.cpp
 *
 *    Copyright 2014 Institute for Dynamic Systems and Control, ETH Zurich
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 *
 *    Designed:    Luca Gherardi
 *    Implemented: Luca Gherardi
 *
 *    @author Luca Gherardi
 */

#include "vehicle_monitor_library/CollisionConstraintChecker.hpp"

#include "vehicle_monitor_library/Vehicle.hpp"
#include "vehicle_monitor_library/VehicleMonitor.hpp"
#include "vehicle_monitor_library/BoundingVolume.hpp"
#include "vehicle_monitor_library/BaseVelocityEstimator.hpp"
#include "vehicle_monitor_library/MotionCaptureSystemFrame.hpp"

using namespace std;

namespace VehicleMonitorLibrary{



CollisionConstraintChecker::CollisionConstraintChecker(
    std::shared_ptr<octomap::OcTree> ocTreePtr,
    BoundingVolume environmentBoundingVolume,
    float maxDistToCheckCollision,
    float collisionThreesholdInBoundingSphereRadius,
    unsigned int projectionWindow,
    unsigned int motionCaptureSystemFrequency,
    std::shared_ptr<BaseVelocityEstimator> velocityEstimator)
:BaseConstraintChecker("COLLISION_CONSTRAINT_CHECKER"),
 _ocTreePtr(ocTreePtr),
 _distMap(maxDistToCheckCollision, _ocTreePtr.get(), environmentBoundingVolume.GetVertexMin(),
          environmentBoundingVolume.GetVertexMax(), true),
          _collisionThreesholdInBoundingSphereRadius(collisionThreesholdInBoundingSphereRadius),
          _projectionWindow(projectionWindow),
          _motionCaptureSystemFrequency(motionCaptureSystemFrequency),
          _velocityEstimator(velocityEstimator){

  if(_velocityEstimator == nullptr){
    throw std::invalid_argument("Velocity Estimator cannot be nullptr");
  }

  _distMap.update();


}

CollisionConstraintChecker::~CollisionConstraintChecker(){

}

bool CollisionConstraintChecker::DoRegisterVehicle(std::shared_ptr<Vehicle> vehiclePtr){

  if(_velocityEstimator->RegisterVehicle(vehiclePtr->GetID())){
    return true;

  }

  return false;

}


bool CollisionConstraintChecker::DoUnregisterVehicle(std::shared_ptr<Vehicle> vehiclePtr){

  if(_velocityEstimator->UnregisterVehicle(vehiclePtr->GetID())){

    return true;

  }

  return false;


}


void CollisionConstraintChecker::DoCheckConstraint(const MotionCaptureSystemFrame& motionCaptureSystemFrame,
                                                   bool emergencyButtonPressed, std::map<std::string, bool>& checkResult) const{


  bool vehicleResult;


  _velocityEstimator->Update(motionCaptureSystemFrame);

  VehicleState frameElement;
  VehicleState estimatedVelocityState;

  float deltaTime = _projectionWindow / (float)_motionCaptureSystemFrequency;


  for(const auto vehicleMapElement : _vehiclesMap){

    // 2) Retrieve current position and estimate velocity

    if( motionCaptureSystemFrame.GetFrameElementForVehicle(vehicleMapElement.first, frameElement) == false){
      continue;
    }

    if(_velocityEstimator->PredictVelocity(vehicleMapElement.second->GetID(),
                                           estimatedVelocityState) == false){
      continue;
    }

    // 1) Project current position according to velocity estimation




    Eigen::Vector3d futurePose(
        frameElement._linear.x() + estimatedVelocityState._linear.x() * deltaTime,
        frameElement._linear.y() + estimatedVelocityState._linear.y() * deltaTime,
        frameElement._linear.z() + estimatedVelocityState._linear.z() * deltaTime
    );

    // 2) Given the future position check for a collision


    octomath::Vector3 futurePoseOctomath(futurePose.x(), futurePose.y(), futurePose.z());

    float distance = _distMap.getDistance(futurePoseOctomath);

    if(distance==DynamicEDTOctomap::distanceValue_Error) {
      std::cout << "VEHICLE NOT IN MAP" << std::endl;
      vehicleResult = false;
    } else {
      vehicleResult =  distance >
      (_collisionThreesholdInBoundingSphereRadius * vehicleMapElement.second->GetBoundingSphereRadius());
    }
    checkResult.insert(make_pair(vehicleMapElement.first, vehicleResult));



  }

}


}
