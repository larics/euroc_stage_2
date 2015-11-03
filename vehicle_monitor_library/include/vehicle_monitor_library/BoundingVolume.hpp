/**
 *    @file VehicleMonitorLibrary/BoundingVolume.hpp
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

#ifndef VML__BOUNDING_VOLUME_H_
#define VML__BOUNDING_VOLUME_H_

#include <octomap/octomap_types.h>
#include <Eigen/Eigen>

namespace VehicleMonitorLibrary {

class BoundingVolume {
 public:
  BoundingVolume(void);
  BoundingVolume(const Eigen::Vector3d& vertexA,
                 const Eigen::Vector3d& vertexB);

  ~BoundingVolume(void);

  octomath::Vector3 getVertexMin() const;
  octomath::Vector3 getVertexMax() const;

  bool isSphereInsideBB(const Eigen::Vector3d& center,
                        double sphereRadius) const;
  bool isOutOfSpace(const Eigen::Vector3d& position) const;

 private:
  Eigen::Vector3d vertex_min_;
  Eigen::Vector3d vertex_max_;
};
}

#endif /* VML__BOUNDING_VOLUME_H_ */
