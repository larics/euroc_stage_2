/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Mar 7, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file FrameRelativeAdapter.cpp
 * @brief Source file for the FrameRelativeAdapter class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <opengv/relative_pose/FrameRelativeAdapter.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/MultiFrame.hpp>

// cameras and distortions
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>

// Constructor.
opengv::relative_pose::FrameRelativeAdapter::FrameRelativeAdapter(
    const okvis::Estimator & estimator,
    const okvis::cameras::NCameraSystem & nCameraSystem, uint64_t multiFrameIdA,
    size_t camIdA, uint64_t multiFrameIdB, size_t camIdB) {

  std::shared_ptr<okvis::MultiFrame> frameAPtr = estimator.multiFrame(
      multiFrameIdA);
  std::shared_ptr<okvis::MultiFrame> frameBPtr = estimator.multiFrame(
      multiFrameIdB);

  bool frameARadialTangential = nCameraSystem.distortionType(camIdA)
      == okvis::cameras::NCameraSystem::RadialTangential;
  bool frameAEquidistant = nCameraSystem.distortionType(camIdA)
      == okvis::cameras::NCameraSystem::Equidistant;
  bool frameBRadialTangential = nCameraSystem.distortionType(camIdB)
      == okvis::cameras::NCameraSystem::RadialTangential;
  bool frameBEquidistant = nCameraSystem.distortionType(camIdB)
      == okvis::cameras::NCameraSystem::Equidistant;

  if ((!frameARadialTangential && !frameAEquidistant)
      || (!frameBRadialTangential && !frameBEquidistant))
    OKVIS_THROW(Exception, "unsupported frame type");

  double fu1 = 0;
  size_t numKeypointsA = frameAPtr->numKeypoints(camIdA);
  if (frameARadialTangential) {
    fu1 = frameAPtr
        ->geometryAs<
            okvis::cameras::PinholeCamera<
                okvis::cameras::RadialTangentialDistortion> >(camIdA)
        ->focalLengthU();
  } else if (frameAEquidistant) {
    fu1 =
        frameAPtr
            ->geometryAs<
                okvis::cameras::PinholeCamera<
                    okvis::cameras::EquidistantDistortion> >(camIdA)
            ->focalLengthU();
  }
  double fu2 = 0.0;
  size_t numKeypointsB = frameBPtr->numKeypoints(camIdB);
  if (frameBRadialTangential) {
    fu2 = frameBPtr
        ->geometryAs<
            okvis::cameras::PinholeCamera<
                okvis::cameras::RadialTangentialDistortion> >(camIdB)
        ->focalLengthU();
  } else if (frameBEquidistant) {
    fu2 =
        frameAPtr
            ->geometryAs<
                okvis::cameras::PinholeCamera<
                    okvis::cameras::EquidistantDistortion> >(camIdB)
            ->focalLengthU();
  }

  // resize members
  bearingVectors1_.resize(numKeypointsA);
  bearingVectors2_.resize(numKeypointsB);
  sigmaAngles1_.resize(numKeypointsA);
  sigmaAngles2_.resize(numKeypointsB);

  matches_.reserve(std::min(numKeypointsA, numKeypointsB));
  std::map<uint64_t, size_t> idMap;
  for (size_t k = 0; k < numKeypointsB; ++k) {

    // get landmark id, if set
    uint64_t lmId = frameBPtr->landmarkId(camIdB, k);
    if (lmId == 0)
      continue;

    // check, if existing
    if (!estimator.isLandmarkAdded(lmId))
      continue;

    // remember it
    idMap.insert(std::pair<uint64_t, size_t>(lmId, k));
  }

  for (size_t k = 0; k < numKeypointsA; ++k) {
    // get landmark id, if set
    uint64_t lmId = frameAPtr->landmarkId(camIdA, k);
    if (lmId == 0)
      continue;

    std::map<uint64_t, size_t>::const_iterator it = idMap.find(lmId);
    if (it != idMap.end()) {
      // whohoo, let's insert it.
      matches_.push_back(okvis::Match(k, it->second, 0.0));
    }
  }

  // precompute
  for (size_t k = 0; k < matches_.size(); ++k) {
    const size_t idx1 = matches_[k].idxA;
    const size_t idx2 = matches_[k].idxB;
    Eigen::Vector2d keypoint;
    double keypointStdDev;
    frameAPtr->getKeypoint(camIdA, idx1, keypoint);
    frameAPtr->getKeypointSize(camIdA, idx1, keypointStdDev);
    keypointStdDev = 0.8 * keypointStdDev / 12.0;
    sigmaAngles1_[idx1] = sqrt(2) * keypointStdDev * keypointStdDev
        / (fu1 * fu1);
    if (frameARadialTangential) {
      frameAPtr
          ->geometryAs<
              okvis::cameras::PinholeCamera<
                  okvis::cameras::RadialTangentialDistortion> >(camIdA)
          ->backProject(keypoint, &bearingVectors1_[idx1]);
      bearingVectors1_[idx1].normalize();
    } else if (frameAEquidistant) {
      frameAPtr
          ->geometryAs<
              okvis::cameras::PinholeCamera<
                  okvis::cameras::EquidistantDistortion> >(camIdA)->backProject(
          keypoint, &bearingVectors1_[idx1]);
      bearingVectors1_[idx1].normalize();
    }

    frameBPtr->getKeypoint(camIdB, idx2, keypoint);
    frameBPtr->getKeypointSize(camIdB, idx2, keypointStdDev);
    keypointStdDev = 0.8 * keypointStdDev / 12.0;
    sigmaAngles2_[idx2] = sqrt(2) * keypointStdDev * keypointStdDev
        / (fu2 * fu2);
    if (frameBRadialTangential) {
      frameBPtr
          ->geometryAs<
              okvis::cameras::PinholeCamera<
                  okvis::cameras::RadialTangentialDistortion> >(camIdB)
          ->backProject(keypoint, &bearingVectors2_[idx2]);
      bearingVectors2_[idx2].normalize();
    } else if (frameBEquidistant) {
      frameBPtr
          ->geometryAs<
              okvis::cameras::PinholeCamera<
                  okvis::cameras::EquidistantDistortion> >(camIdB)->backProject(
          keypoint, &bearingVectors2_[idx2]);
      bearingVectors2_[idx2].normalize();
    }
  }
}

// Retrieve the bearing vector of a correspondence in viewpoint 1.
opengv::bearingVector_t opengv::relative_pose::FrameRelativeAdapter::getBearingVector1(
    size_t index) const {
  return bearingVectors1_[matches_[index].idxA];
}

// Retrieve the bearing vector of a correspondence in viewpoint 2.
opengv::bearingVector_t opengv::relative_pose::FrameRelativeAdapter::getBearingVector2(
    size_t index) const {
  return bearingVectors2_[matches_[index].idxB];
}

// Retrieve the position of a camera of a correspondence in viewpoint 1 seen from the origin of the viewpoint.
opengv::translation_t opengv::relative_pose::FrameRelativeAdapter::getCamOffset1(
    size_t /*index*/) const {
  //We could also check here for camIndex being 0, because this adapter is made
  //for a single camera only
  return Eigen::Vector3d::Zero();
}

// Retrieve the rotation from a camera of a correspondence in viewpoint 1 to the viewpoint origin.
opengv::rotation_t opengv::relative_pose::FrameRelativeAdapter::getCamRotation1(
    size_t /*index*/) const {
  //We could also check here for camIndex being 0, because this adapter is made
  //for a single camera only
  return Eigen::Matrix3d::Identity();
}

// Retrieve the position of a camera of a correspondence in viewpoint 2 seen from the origin of the viewpoint.
opengv::translation_t opengv::relative_pose::FrameRelativeAdapter::getCamOffset2(
    size_t /*index*/) const {
  //We could also check here for camIndex being 0, because this adapter is made
  //for a single camera only
  return Eigen::Vector3d::Zero();
}

// Retrieve the rotation from a camera of a correspondence in viewpoint 2 to the viewpoint origin.
opengv::rotation_t opengv::relative_pose::FrameRelativeAdapter::getCamRotation2(
    size_t /*index*/) const {
  //We could also check here for camIndex being 0, because this adapter is made
  //for a single camera only
  return Eigen::Matrix3d::Identity();
}

// Retrieve the number of correspondences.
size_t opengv::relative_pose::FrameRelativeAdapter::getNumberCorrespondences() const {
  return matches_.size();
}

// Obtain the angular standard deviation of the correspondence in frame 1 in [rad].
double opengv::relative_pose::FrameRelativeAdapter::getSigmaAngle1(
    size_t index) {
  return sigmaAngles1_[matches_[index].idxA];
}

// Obtain the angular standard deviation of the correspondence in frame 2 in [rad].
double opengv::relative_pose::FrameRelativeAdapter::getSigmaAngle2(
    size_t index) {
  return sigmaAngles2_[matches_[index].idxB];
}