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
 *  Created on: Jun 17, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file VioParametersReader.cpp
 * @brief Source file for the VioParametersReader class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <algorithm>

#include <glog/logging.h>

#include <okvis/cameras/NCameraSystem.hpp>
#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>

#include <opencv2/core/core.hpp>

#include <okvis/VioParametersReader.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

// The default constructor.
VioParametersReader::VioParametersReader()
    : useDriver(false),
      readConfigFile_(false) {
  vioParameters_.publishing.publishRate = 0;
}

// The constructor. This calls readConfigFile().
VioParametersReader::VioParametersReader(const std::string& filename) {
  // reads
  readConfigFile(filename);
}

// Read and parse a config file.
void VioParametersReader::readConfigFile(const std::string& filename) {
  vioParameters_.optimization.useMedianFilter = false;
  vioParameters_.optimization.timeReserve.fromSec(0.005);

  // reads
  cv::FileStorage file(filename, cv::FileStorage::READ);

  OKVIS_ASSERT_TRUE(Exception, file.isOpened(),
                    "Could not open config file: " << filename);
  LOG(INFO) << "Opened configuration file: " << filename;

  // do we use the IMU?
  bool success = parseBoolean(file["useImu"],
                              vioParameters_.optimization.useImu);
  OKVIS_ASSERT_TRUE(Exception, success,
                    "'useImu' parameter missing in configuration file.");

  // number of keyframes
  if (file["numKeyframes"].isInt()) {
    file["numKeyframes"] >> vioParameters_.optimization.numKeyframes;
  } else {
    LOG(WARNING)
        << "numKeyframes parameter not provided. Setting to default numKeyframes=5.";
    vioParameters_.optimization.numKeyframes = 5;
  }
  // number of IMU frames
  if (file["numImuFrames"].isInt()) {
    file["numImuFrames"] >> vioParameters_.optimization.numImuFrames;
  } else {
    LOG(WARNING)
        << "numImuFrames parameter not provided. Setting to default numImuFrames=2.";
    vioParameters_.optimization.numImuFrames = 2;
  }
  // minimum ceres iterations
  if (file["ceres_options"]["minIterations"].isInt()) {
    file["ceres_options"]["minIterations"]
        >> vioParameters_.optimization.min_iterations;
  } else {
    LOG(WARNING)
        << "ceres_options: minIterations parameter not provided. Setting to default minIterations=1";
    vioParameters_.optimization.min_iterations = 1;
  }
  // maximum ceres iterations
  if (file["ceres_options"]["maxIterations"].isInt()) {
    file["ceres_options"]["maxIterations"]
        >> vioParameters_.optimization.max_iterations;
  } else {
    LOG(WARNING)
        << "ceres_options: maxIterations parameter not provided. Setting to default maxIterations=10.";
    vioParameters_.optimization.max_iterations = 10;
  }
  // ceres time limit
  if (file["ceres_options"]["timeLimit"].isReal()) {
    file["ceres_options"]["timeLimit"] >> vioParameters_.optimization.timeLimitForMatchingAndOptimization;
  } else {
    LOG(WARNING)
        << "ceres_options: timeLimit parameter not provided. Setting no time limit.";
    vioParameters_.optimization.timeLimitForMatchingAndOptimization = -1.0;
  }

  // do we use the direct driver?
  success = parseBoolean(file["useDriver"], useDriver);
  OKVIS_ASSERT_TRUE(Exception, success,
                    "'useDriver' parameter missing in configuration file.");

  // do we save the graphs?
  success = parseBoolean(file["saveGraphs"],
                         vioParameters_.visualization.saveGraphs);
  OKVIS_ASSERT_TRUE(Exception, success,
                    "'saveGraphs' parameter missing in configuration file.");

  // display images?
  success = parseBoolean(file["displayImages"],
                         vioParameters_.visualization.displayImages);
  OKVIS_ASSERT_TRUE(Exception, success,
                    "'displayImages' parameter missing in configuration file.");

  // detection threshold
  success = file["detectionThreshold"].isReal();
  OKVIS_ASSERT_TRUE(
      Exception, success,
      "'detectionThreshold' parameter missing in configuration file.");
  file["detectionThreshold"] >> vioParameters_.optimization.detectionThreshold;

  // detection octaves
  success = file["detectionOctaves"].isInt();
  OKVIS_ASSERT_TRUE(
      Exception, success,
      "'detectionOctaves' parameter missing in configuration file.");
  file["detectionOctaves"] >> vioParameters_.optimization.detectionOctaves;
  OKVIS_ASSERT_TRUE(Exception,
                    vioParameters_.optimization.detectionOctaves >= 0,
                    "Invalid parameter value.");

  // image delay
  success = file["imageDelay"].isReal();
  OKVIS_ASSERT_TRUE(Exception, success,
                    "'imageDelay' parameter missing in configuration file.");
  file["imageDelay"] >> vioParameters_.sensors_information.imageDelay;
  LOG(INFO) << "imageDelay=" << vioParameters_.sensors_information.imageDelay;

  // camera rate
  success = file["camera_params"]["camera_rate"].isInt();
  OKVIS_ASSERT_TRUE(
      Exception, success,
      "'camera_params: camera_rate' parameter missing in configuration file.");
  file["camera_params"]["camera_rate"]
      >> vioParameters_.sensors_information.cameraRate;

  // timestamp tolerance
  if (file["camera_params"]["timestamp_tolerance"].isReal()) {
    file["camera_params"]["timestamp_tolerance"]
        >> vioParameters_.sensors_information.frameTimestampTolerance;
    OKVIS_ASSERT_TRUE(
        Exception,
        vioParameters_.sensors_information.frameTimestampTolerance
            < 0.5 / vioParameters_.sensors_information.cameraRate,
        "Timestamp tolerance for stereo frames is larger than half the time between frames.");
    OKVIS_ASSERT_TRUE(
        Exception,
        vioParameters_.sensors_information.frameTimestampTolerance >= 0.0,
        "Timestamp tolerance is smaller than 0");
  } else {
    vioParameters_.sensors_information.frameTimestampTolerance = 0.2
        / vioParameters_.sensors_information.cameraRate;
    LOG(WARNING)
        << "No timestamp tolerance for stereo frames specified. Setting to "
        << vioParameters_.sensors_information.frameTimestampTolerance;
  }

  // camera params
  if (file["camera_params"]["sigma_absolute_translation"].isReal()) {
    file["camera_params"]["sigma_absolute_translation"]
        >> vioParameters_.camera_extrinsics.sigma_absolute_translation;
  } else {
    vioParameters_.camera_extrinsics.sigma_absolute_translation = 0.0;
    LOG(WARNING)
        << "camera_params: sigma_absolute_translation parameter not provided. Setting to default 0.0";
  }
  if (file["camera_params"]["sigma_absolute_orientation"].isReal()) {
    file["camera_params"]["sigma_absolute_orientation"]
        >> vioParameters_.camera_extrinsics.sigma_absolute_orientation;
  } else {
    vioParameters_.camera_extrinsics.sigma_absolute_orientation = 0.0;
    LOG(WARNING)
        << "camera_params: sigma_absolute_orientation parameter not provided. Setting to default 0.0";
  }
  if (file["camera_params"]["sigma_c_relative_translation"].isReal()) {
    file["camera_params"]["sigma_c_relative_translation"]
        >> vioParameters_.camera_extrinsics.sigma_c_relative_translation;
  } else {
    vioParameters_.camera_extrinsics.sigma_c_relative_translation = 0.0;
    LOG(WARNING)
        << "camera_params: sigma_c_relative_translation parameter not provided. Setting to default 0.0";
  }
  if (file["camera_params"]["sigma_c_relative_orientation"].isReal()) {
    file["camera_params"]["sigma_c_relative_orientation"]
        >> vioParameters_.camera_extrinsics.sigma_c_relative_orientation;
  } else {
    vioParameters_.camera_extrinsics.sigma_c_relative_orientation = 0.0;
    LOG(WARNING)
        << "camera_params: sigma_c_relative_orientation parameter not provided. Setting to default 0.0";
  }

  success = file["publish_rate"].isInt();
  OKVIS_ASSERT_TRUE(Exception, success,
                    "'publish_rate' parameter missing in configuration file.");
  file["publish_rate"] >> vioParameters_.publishing.publishRate;

  vioParameters_.publishing.landmarkQualityThreshold = 1.0e-5;
  if (file["publishing_options"]["landmarkQualityThreshold"].isReal()) {
    file["publishing_options"]["landmarkQualityThreshold"]
        >> vioParameters_.publishing.landmarkQualityThreshold;
  }

  vioParameters_.publishing.maxLandmarkQuality = 0.05;
  if (file["publishing_options"]["maximumLandmarkQuality"].isReal()) {
    file["publishing_options"]["maximumLandmarkQuality"]
        >> vioParameters_.publishing.maxLandmarkQuality;
  }

  vioParameters_.publishing.maxPathLength = 20;
  if (file["publishing_options"]["maxPathLength"].isInt()) {
    vioParameters_.publishing.maxPathLength =
        (int) (file["publishing_options"]["maxPathLength"]);
  }

  vioParameters_.publishing.publishImuPropagatedState = true;
  if (!parseBoolean(file["publishing_options"]["publishImuPropagatedState"],
                   vioParameters_.publishing.publishImuPropagatedState)) {
    LOG(WARNING) << "publishing_options: publishImuPropagatedState parameter not provided. Setting to default 'true'";
  }

  // camera calibration
  std::vector<okvis::VioParametersReader::CameraCalibration> calibrations;
  if(!getCameraCalibration(calibrations, file))
    LOG(FATAL) << "Did not find any calibration!";

  size_t camIdx = 0;
  for (size_t i = 0; i < calibrations.size(); ++i) {

    std::shared_ptr<const okvis::kinematics::Transformation> T_SC_okvis_ptr(
          new okvis::kinematics::Transformation(calibrations[i].T_SC.r(),
                                                calibrations[i].T_SC.q().normalized()));

    if (strcmp(calibrations[i].distortionType.c_str(), "equidistant") == 0 ||
        strcmp(calibrations[i].distortionType.c_str(), "equi") == 0) {
      vioParameters_.nCameraSystem.addCamera(
          T_SC_okvis_ptr,
          std::shared_ptr<const okvis::cameras::CameraBase>(
              new okvis::cameras::PinholeCamera<
                  okvis::cameras::EquidistantDistortion>(
                  calibrations[i].imageDimension[0],
                  calibrations[i].imageDimension[1],
                  calibrations[i].focalLength[0],
                  calibrations[i].focalLength[1],
                  calibrations[i].principalPoint[0],
                  calibrations[i].principalPoint[1],
                  okvis::cameras::EquidistantDistortion(
                    calibrations[i].distortionCoefficients[0],
                    calibrations[i].distortionCoefficients[1],
                    calibrations[i].distortionCoefficients[2],
                    calibrations[i].distortionCoefficients[3])/*, id ?*/)),
          okvis::cameras::NCameraSystem::Equidistant/*, computeOverlaps ?*/);
      std::stringstream s;
      s << calibrations[i].T_SC.T();
      LOG(INFO) << "Equidistant pinhole camera " << camIdx
                << " with T_SC=\n" << s.str();
    } else if (strcmp(calibrations[i].distortionType.c_str(), "radialtangential") == 0
               || strcmp(calibrations[i].distortionType.c_str(), "plumb_bob") == 0) {
      vioParameters_.nCameraSystem.addCamera(
          T_SC_okvis_ptr,
          std::shared_ptr<const okvis::cameras::CameraBase>(
              new okvis::cameras::PinholeCamera<
                  okvis::cameras::RadialTangentialDistortion>(
                  calibrations[i].imageDimension[0],
                  calibrations[i].imageDimension[1],
                  calibrations[i].focalLength[0],
                  calibrations[i].focalLength[1],
                  calibrations[i].principalPoint[0],
                  calibrations[i].principalPoint[1],
                  okvis::cameras::RadialTangentialDistortion(
                    calibrations[i].distortionCoefficients[0],
                    calibrations[i].distortionCoefficients[1],
                    calibrations[i].distortionCoefficients[2],
                    calibrations[i].distortionCoefficients[3])/*, id ?*/)),
          okvis::cameras::NCameraSystem::RadialTangential/*, computeOverlaps ?*/);
      std::stringstream s;
      s << calibrations[i].T_SC.T();
      LOG(INFO) << "Radial tangential pinhole camera " << camIdx
                << " with T_SC=\n" << s.str();
    } else if (strcmp(calibrations[i].distortionType.c_str(), "radialtangential8") == 0
               || strcmp(calibrations[i].distortionType.c_str(), "plumb_bob8") == 0) {
      vioParameters_.nCameraSystem.addCamera(
          T_SC_okvis_ptr,
          std::shared_ptr<const okvis::cameras::CameraBase>(
              new okvis::cameras::PinholeCamera<
                  okvis::cameras::RadialTangentialDistortion8>(
                  calibrations[i].imageDimension[0],
                  calibrations[i].imageDimension[1],
                  calibrations[i].focalLength[0],
                  calibrations[i].focalLength[1],
                  calibrations[i].principalPoint[0],
                  calibrations[i].principalPoint[1],
                  okvis::cameras::RadialTangentialDistortion8(
                    calibrations[i].distortionCoefficients[0],
                    calibrations[i].distortionCoefficients[1],
                    calibrations[i].distortionCoefficients[2],
                    calibrations[i].distortionCoefficients[3],
                    calibrations[i].distortionCoefficients[4],
                    calibrations[i].distortionCoefficients[5],
                    calibrations[i].distortionCoefficients[6],
                    calibrations[i].distortionCoefficients[7])/*, id ?*/)),
          okvis::cameras::NCameraSystem::RadialTangential8/*, computeOverlaps ?*/);
      std::stringstream s;
      s << calibrations[i].T_SC.T();
      LOG(INFO) << "Radial tangential 8 pinhole camera " << camIdx
                << " with T_SC=\n" << s.str();
    } else {
      LOG(ERROR) << "unrecognized distortion type " << calibrations[i].distortionType;
    }
    ++camIdx;
  }

  vioParameters_.sensors_information.imuIdx = 0;

  cv::FileNode T_BS_ = file["T_BS"];
  OKVIS_ASSERT_TRUE(
      Exception,
      T_BS_.isSeq(),
      "'T_BS' parameter missing in the configuration file or in the wrong format.")

  Eigen::Matrix4d T_BS_e;
  T_BS_e << T_BS_[0], T_BS_[1], T_BS_[2], T_BS_[3], T_BS_[4], T_BS_[5], T_BS_[6], T_BS_[7], T_BS_[8], T_BS_[9], T_BS_[10], T_BS_[11], T_BS_[12], T_BS_[13], T_BS_[14], T_BS_[15];

  vioParameters_.imu.T_BS = okvis::kinematics::Transformation(T_BS_e);
  std::stringstream s;
  s << vioParameters_.imu.T_BS.T();
  LOG(INFO) << "IMU with transformation T_BS=\n" << s.str();

  // the IMU parameters
  cv::FileNode imu_params = file["imu_params"];
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["a_max"].isReal(),
      "'imu_params: a_max' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["g_max"].isReal(),
      "'imu_params: g_max' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["sigma_g_c"].isReal(),
      "'imu_params: sigma_g_c' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["sigma_a_c"].isReal(),
      "'imu_params: sigma_a_c' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["sigma_gw_c"].isReal(),
      "'imu_params: sigma_gw_c' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["sigma_g_c"].isReal(),
      "'imu_params: sigma_g_c' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["tau"].isReal(),
      "'imu_params: tau' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(Exception, imu_params["g"].isReal(),
                    "'imu_params: g' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(Exception, imu_params["a0"].isSeq(),
                    "'imu_params: a0' parameter missing in configuration file.");
  OKVIS_ASSERT_TRUE(
      Exception, imu_params["imu_rate"].isInt(),
      "'imu_params: imu_rate' parameter missing in configuration file.");
  imu_params["a_max"] >> vioParameters_.imu.a_max;
  imu_params["g_max"] >> vioParameters_.imu.g_max;
  imu_params["sigma_g_c"] >> vioParameters_.imu.sigma_g_c;
  imu_params["sigma_a_c"] >> vioParameters_.imu.sigma_a_c;
  imu_params["sigma_gw_c"] >> vioParameters_.imu.sigma_gw_c;
  imu_params["sigma_aw_c"] >> vioParameters_.imu.sigma_aw_c;
  imu_params["imu_rate"] >> vioParameters_.imu.rate;
  imu_params["tau"] >> vioParameters_.imu.tau;
  imu_params["g"] >> vioParameters_.imu.g;

  vioParameters_.imu.a0 = Eigen::Vector3d((double) (imu_params["a0"][0]),
                                          (double) (imu_params["a0"][1]),
                                          (double) (imu_params["a0"][2]));
  readConfigFile_ = true;
}

// Parses booleans from a cv::FileNode. OpenCV sadly has no implementation like this.
bool VioParametersReader::parseBoolean(cv::FileNode node, bool& val) const {
  if (node.isInt()) {
    val = (int) (node) != 0;
    return true;
  }
  if (node.isString()) {
    std::string str = (std::string) (node);
    // cut out first word. str currently contains everything including comments
    str = str.substr(0,str.find(" "));
    // transform it to all lowercase
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    /* from yaml.org/type/bool.html:
     * Booleans are formatted as English words
     * (“true”/“false”, “yes”/“no” or “on”/“off”)
     * for readability and may be abbreviated as
     * a single character “y”/“n” or “Y”/“N”. */
    if (str.compare("false")  == 0
        || str.compare("no")  == 0
        || str.compare("n")   == 0
        || str.compare("off") == 0) {
      val = false;
      return true;
    }
    if (str.compare("true")   == 0
        || str.compare("yes") == 0
        || str.compare("y")   == 0
        || str.compare("on")  == 0) {
      val = true;
      return true;
    }
  }
  return false;
}

bool VioParametersReader::getCameraCalibration(
    std::vector<okvis::VioParametersReader::CameraCalibration>& calibrations,
    cv::FileStorage& configurationFile) {

  bool success = getCalibrationViaConfig(calibrations, configurationFile["cameras"]);

#ifdef HAVE_LIBVISENSOR
  if (useDriver && !success) {
    // start up sensor
    viSensor = std::shared_ptr<visensor::ViSensorDriver>(
          new visensor::ViSensorDriver());
    try {
      // use autodiscovery to find sensor. TODO: specify IP in config?
      viSensor->init();
    } catch (Exception const &ex) {
      LOG(ERROR) << ex.what();
      exit(1);
    }

    success = getCalibrationViaVisensorAPI(calibrations);
  }
#endif

  return success;
}

// Get the camera calibration via the configuration file.
bool VioParametersReader::getCalibrationViaConfig(
    std::vector<okvis::VioParametersReader::CameraCalibration>& calibrations,
    cv::FileNode cameraNode) const {

  calibrations.clear();
  bool gotCalibration = false;
  // first check if calibration is available in config file
  if (cameraNode.isSeq()
     && cameraNode.size() > 0) {
    size_t camIdx = 0;
    for (cv::FileNodeIterator it = cameraNode.begin();
        it != cameraNode.end(); ++it) {
      if ((*it).isMap()
          && (*it)["T_SC"].isSeq()
          && (*it)["image_dimension"].isSeq()
          && (*it)["image_dimension"].size() == 2
          && (*it)["distortion_coefficients"].isSeq()
          && (*it)["distortion_coefficients"].size() >= 4
          && (*it)["distortion_type"].isString()
          && (*it)["focal_length"].isSeq()
          && (*it)["focal_length"].size() == 2
          && (*it)["principal_point"].isSeq()
          && (*it)["principal_point"].size() == 2) {
        LOG(INFO) << "Found calibration in configuration file for camera " << camIdx;
        gotCalibration = true;
      } else {
        LOG(WARNING) << "Found incomplete calibration in configuration file for camera " << camIdx
                     << ". Will not use the calibration from the configuration file.";
        return false;
      }
      ++camIdx;
    }
  }
  else
    LOG(INFO) << "Did not find a calibration in the configuration file.";

  if (gotCalibration) {
    for (cv::FileNodeIterator it = cameraNode.begin();
        it != cameraNode.end(); ++it) {

      CameraCalibration calib;

      cv::FileNode T_SC_node = (*it)["T_SC"];
      cv::FileNode imageDimensionNode = (*it)["image_dimension"];
      cv::FileNode distortionCoefficientNode = (*it)["distortion_coefficients"];
      cv::FileNode focalLengthNode = (*it)["focal_length"];
      cv::FileNode principalPointNode = (*it)["principal_point"];

      // extrinsics
      Eigen::Matrix4d T_SC;
      T_SC << T_SC_node[0], T_SC_node[1], T_SC_node[2], T_SC_node[3], T_SC_node[4], T_SC_node[5], T_SC_node[6], T_SC_node[7], T_SC_node[8], T_SC_node[9], T_SC_node[10], T_SC_node[11], T_SC_node[12], T_SC_node[13], T_SC_node[14], T_SC_node[15];
      calib.T_SC = okvis::kinematics::Transformation(T_SC);

      calib.imageDimension << imageDimensionNode[0], imageDimensionNode[1];
      calib.distortionCoefficients.resize(distortionCoefficientNode.size());
      for(size_t i=0; i<distortionCoefficientNode.size(); ++i) {
        calib.distortionCoefficients[i] = distortionCoefficientNode[i];
      }
      calib.focalLength << focalLengthNode[0], focalLengthNode[1];
      calib.principalPoint << principalPointNode[0], principalPointNode[1];
      calib.distortionType = (std::string)((*it)["distortion_type"]);

      calibrations.push_back(calib);
    }
  }
  return gotCalibration;
}

// Get the camera calibrations via the visensor API.
bool VioParametersReader::getCalibrationViaVisensorAPI(
    std::vector<okvis::VioParametersReader::CameraCalibration>& calibrations) const{
#ifdef HAVE_LIBVISENSOR
  if (viSensor == nullptr) {
    LOG(ERROR) << "Tried to get calibration from the sensor. But the sensor is not set up.";
    return false;
  }

  calibrations.clear();

  std::vector<visensor::SensorId::SensorId> listOfCameraIds =
      viSensor->getListOfCameraIDs();

  for (auto it = listOfCameraIds.begin(); it != listOfCameraIds.end(); ++it) {
    visensor::ViCameraCalibration calibrationFromAPI;
    okvis::VioParametersReader::CameraCalibration calibration;

    // TODO(burrimi): move this to some sort of configuration file.
    const int is_flipped = 1;
    const int slot_number = 0;

    std::vector<visensor::ViCameraCalibration> calibration_list =
        viSensor->getCameraCalibrations(*it, slot_number, is_flipped, visensor::ViCameraLensModel::LensModelTypes::EQUIDISTANT,
                                        visensor::ViCameraProjectionModel::ProjectionModelTypes::PINHOLE);

    // We prefer equidistant, therefore we only try to load a radial model if no equidistant is found.
    if(calibrations.empty()) {
      calibration_list = viSensor->getCameraCalibrations(*it, slot_number, is_flipped, visensor::ViCameraLensModel::LensModelTypes::RADTAN,
                                                         visensor::ViCameraProjectionModel::ProjectionModelTypes::PINHOLE);
    }

    if(calibration_list.empty()) {
      LOG(ERROR) << "Reading the calibration via the sensor API failed.";
      calibrations.clear();
      return false;
    }

    calibrationFromAPI = calibration_list.front();

    LOG(INFO) << "Reading the calbration for camera " << size_t(*it) << " via API successful";
    std::vector<double> R = calibrationFromAPI.R_;
    std::vector<double> t = calibrationFromAPI.t_;
    // getCameraCalibration apparently gives T_CI back.
    //(Confirmed by comparing it to output of service)
    Eigen::Matrix4d T_CI;
    T_CI << R[0], R[1], R[2], t[0],
        R[3], R[4], R[5], t[1],
        R[6], R[7], R[8], t[2],
        0,    0,    0,    1;
    okvis::kinematics::Transformation T_CI_okvis(T_CI);
    calibration.T_SC = T_CI_okvis.inverse();

    calibration.imageDimension << calibrationFromAPI.resolution_[0], calibrationFromAPI.resolution_[1];

    if(calibrationFromAPI.projection_model_->type_ != visensor::ViCameraProjectionModel::ProjectionModelTypes::PINHOLE) {
      LOG(ERROR) << "Reading the calibration via the sensor API failed. Projection model not supported";
      calibrations.clear();
      return false;
    }
    std::vector<double> projection_coefficients = calibrationFromAPI.projection_model_->getCoefficients();

    calibration.focalLength << projection_coefficients[0],
        projection_coefficients[1];
    calibration.principalPoint << projection_coefficients[2],
        projection_coefficients[3];


    if(calibrationFromAPI.lens_model_->type_ != visensor::ViCameraLensModel::LensModelTypes::EQUIDISTANT) {
      std::vector<double> lens_coefficients = calibrationFromAPI.projection_model_->getCoefficients();

      calibration.distortionCoefficients << lens_coefficients[0],
          lens_coefficients[1],
          lens_coefficients[2],
          lens_coefficients[3];
      calibration.distortionType = "equdistant";
    } else if(calibrationFromAPI.lens_model_->type_ != visensor::ViCameraLensModel::LensModelTypes::RADTAN) {
      std::vector<double> lens_coefficients = calibrationFromAPI.projection_model_->getCoefficients();

      calibration.distortionCoefficients << lens_coefficients[0],
          lens_coefficients[1],
          lens_coefficients[2],
          lens_coefficients[3];
      calibration.distortionType = "radialtangential";
    } else {
      LOG(ERROR) << "Reading the calibration via the sensor API failed. Lense model not supported";
      calibrations.clear();
      return false;
    }

    calibrations.push_back(calibration);
  }

  return calibrations.empty() == false;
#else
  static_cast<void>(calibrations); // unused
  LOG(ERROR) << "Tried to get calibration directly from the sensor. However libvisensor was not found.";
  return false;
#endif
}


}  // namespace okvis
