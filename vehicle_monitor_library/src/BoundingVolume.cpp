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

namespace VehicleMonitorLibrary {

// default constructor
BoundingVolume::BoundingVolume() : vertex_min_(), vertex_max_() {}

BoundingVolume::~BoundingVolume(){

};

BoundingVolume::BoundingVolume(const Eigen::Vector3d& vertexA,
                               const Eigen::Vector3d& vertexB) {
  double minX = min(vertexA.x(), vertexB.x());
  double maxX = max(vertexA.x(), vertexB.x());
  double minY = min(vertexA.y(), vertexB.y());
  double maxY = max(vertexA.y(), vertexB.y());
  double minZ = min(vertexA.z(), vertexB.z());
  double maxZ = max(vertexA.z(), vertexB.z());

  Eigen::Vector3d minVertex(minX, minY, minZ);
  Eigen::Vector3d maxVertex(maxX, maxY, maxZ);

  vertex_min_ = vertexA;
  vertex_max_ = maxVertex;
}

octomath::Vector3 BoundingVolume::getVertexMin() const {
  octomath::Vector3 vertexMin(vertex_min_.x(), vertex_min_.y(),
                              vertex_min_.z());

  return vertexMin;
}

octomath::Vector3 BoundingVolume::getVertexMax() const {
  octomath::Vector3 vertexMax(vertex_max_.x(), vertex_max_.y(),
                              vertex_max_.z());

  return vertexMax;
}

bool BoundingVolume::isSphereInsideBB(const Eigen::Vector3d& center,
                                      double sphereRadius) const {
  double sqrDist = 0;

  if ((center.x() - sphereRadius) < vertex_min_.x()) {
    return false;
  }

  if ((center.x() + sphereRadius) > vertex_max_.x()) {
    return false;
  }

  if ((center.y() - sphereRadius) < vertex_min_.y()) {
    return false;
  }

  if ((center.y() + sphereRadius) > vertex_max_.y()) {
    return false;
  }

  if ((center.z() - sphereRadius) < vertex_min_.z()) {
    return false;
  }

  if ((center.z() + sphereRadius) > vertex_max_.z()) {
    return false;
  }

  return true;
}

/**
 * Checks if the provided box (BoundingVolume) is outside the BoundingBox
 * (=allowed space)
 */
bool BoundingVolume::isOutOfSpace(const Eigen::Vector3d& posistion) const {
  if ((posistion.x() < vertex_min_.x() || posistion.x() > vertex_max_.x()) ||
      (posistion.y() < vertex_min_.y() || posistion.y() > vertex_max_.y()) ||
      (posistion.z() < vertex_min_.z() || posistion.z() > vertex_max_.z())) {
    return true;
  }

  return false;
}
}
