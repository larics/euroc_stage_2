/**
 *    @file BoundingVolume.cpp
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
 *    Implemented: Caspar Reist and Luca Gherardi
 *
 *    @author Luca Gherardi
 */

#include "vehicle_monitor_library/BoundingVolume.hpp"

#include <algorithm>

using namespace std;

namespace VehicleMonitorLibrary{

//default constructor
BoundingVolume::BoundingVolume() :
				    _vertexMin(),
				    _vertexMax(){

}

BoundingVolume::~BoundingVolume(){

};

BoundingVolume::BoundingVolume(const Eigen::Vector3d& vertexA, const Eigen::Vector3d& vertexB) {

  float minX = min(vertexA.x(), vertexB.x());
  float maxX = max(vertexA.x(), vertexB.x());
  float minY = min(vertexA.y(), vertexB.y());
  float maxY = max(vertexA.y(), vertexB.y());
  float minZ = min(vertexA.z(), vertexB.z());
  float maxZ = max(vertexA.z(), vertexB.z());

  Eigen::Vector3d minVertex(minX, minY, minZ);
  Eigen::Vector3d maxVertex(maxX, maxY, maxZ);

  _vertexMin = vertexA;
  _vertexMax = maxVertex;
}

octomath::Vector3 BoundingVolume::GetVertexMin() const{

  octomath::Vector3 vertexMin(_vertexMin.x(), _vertexMin.y(), _vertexMin.z());

  return vertexMin;

}

octomath::Vector3 BoundingVolume::GetVertexMax() const{

  octomath::Vector3 vertexMax(_vertexMax.x(), _vertexMax.y(), _vertexMax.z());

  return vertexMax;

}

bool BoundingVolume::IsSphereInsideBB(const Eigen::Vector3d& center, float sphereRadius) const{

  float sqrDist = 0;

  if( (center.x() - sphereRadius) < _vertexMin.x()){
    return false;
  }

  if( (center.x() + sphereRadius) > _vertexMax.x()){
    return false;
  }

  if( (center.y() - sphereRadius) < _vertexMin.y()){
    return false;
  }

  if( (center.y() + sphereRadius) > _vertexMax.y()){
    return false;
  }

  if( (center.z() - sphereRadius) < _vertexMin.z()){
    return false;
  }

  if( (center.z() + sphereRadius) > _vertexMax.z()){
    return false;
  }

  return true;

}

/**
 * Checks if the provided box (BoundingVolume) is outside the BoundingBox (=allowed space)
 */
bool BoundingVolume::IsOutOfSpace(const Eigen::Vector3d& posistion) const{

  if ((posistion.x() < _vertexMin.x() || posistion.x() > _vertexMax.x()) ||
      (posistion.y() < _vertexMin.y() || posistion.y() > _vertexMax.y()) ||
      (posistion.z() < _vertexMin.z() || posistion.z() > _vertexMax.z())){

    return true;

  }

  return false;

}


}
