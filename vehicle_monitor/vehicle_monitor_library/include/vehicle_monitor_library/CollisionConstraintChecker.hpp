/**
 *    @file VehicleMonitorLibrary/CollisionConstraintChecker.hpp
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

#ifndef VML__COLLISION_CONSTRAINT_CHECKER_H_
#define VML__COLLISION_CONSTRAINT_CHECKER_H_

#include "BaseConstraintChecker.hpp"

#include "BoundingVolume.hpp"

#include "octomap/octomap_types.h"
#include "dynamicEDT3D/dynamicEDTOctomap.h"

namespace octomap{
class OcTree;
}

namespace VehicleMonitorLibrary{

class BaseVelocityEstimator;

class CollisionConstraintChecker : public BaseConstraintChecker {

 public:

  CollisionConstraintChecker(
      std::shared_ptr<octomap::OcTree> ocTreePtr,
      BoundingVolume environmentBoundingVolume,
      float maxDistToCheckCollision,
      float collisionThreesholdInBoundingSphereRadius,
      unsigned int projectionWindow,
      unsigned int motionCaptureSystemFrequency,
      std::shared_ptr<BaseVelocityEstimator> velocityEstimator);

  virtual ~CollisionConstraintChecker();

 protected:

  virtual bool DoRegisterVehicle(std::shared_ptr<Vehicle> vehiclePtr);

  virtual bool DoUnregisterVehicle(std::shared_ptr<Vehicle> vehiclePtr);

  virtual void DoCheckConstraint(const MotionCaptureSystemFrame& motionCaptureSystemFrame,
                                 bool emergencyButtonPressed, std::map<std::string, bool>& checkResult) const;

 private:

  std::shared_ptr<octomap::OcTree> _ocTreePtr;

  DynamicEDTOctomap _distMap;

  float _collisionThreesholdInBoundingSphereRadius;

  // number of time steps used to compute the future position of a vehicle
  unsigned int _projectionWindow;

  unsigned int _motionCaptureSystemFrequency;

  std::shared_ptr<BaseVelocityEstimator> _velocityEstimator;

};

}
#endif /* VML__COLLISION_CONSTRAINT_CHECKER_H_ */
