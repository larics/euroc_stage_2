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
 *  Created on: Mar 23, 2012
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file Subscriber.cpp
 * @brief Source file for the Subscriber class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <glog/logging.h>
#include <okvis/Subscriber.hpp>
#include <functional>

#define THRESHOLD_DATA_DELAY_WARNING 0.1 // in seconds

/// \brief okvis Main namespace of this package.
namespace okvis {

Subscriber::~Subscriber()
{
  if (imgTransport_ != 0)
    delete imgTransport_;
}

Subscriber::Subscriber(ros::NodeHandle& nh, okvis::VioInterface* vioInterfacePtr,
                       const okvis::VioParametersReader& param_reader)
    : vioInterface_(vioInterfacePtr)
{
  param_reader.getParameters(vioParameters_);
  imgTransport_ = 0;
  if (param_reader.useDriver) {
#ifdef HAVE_LIBVISENSOR
    if(param_reader.viSensor != nullptr)
      sensor_ = param_reader.viSensor;
    initialiseDriverCallbacks();
    std::vector<unsigned int> camRate(
        vioParameters_.nCameraSystem.numCameras(),
        vioParameters_.sensors_information.cameraRate);
    startSensors(camRate, vioParameters_.imu.rate);
#else
    LOG(ERROR) << "Configuration specified to directly access the driver. "
               << "However the visensor library was not found. Trying to set up ROS nodehandle instead";
    setNodeHandle(nh);
#endif
  } else
    setNodeHandle(nh);
}

void Subscriber::setNodeHandle(ros::NodeHandle& nh)
{

  nh_ = &nh;

  imageSubscribers_.resize(vioParameters_.nCameraSystem.numCameras());

  // set up image reception
  if (imgTransport_ != 0)
    delete imgTransport_;
  imgTransport_ = new image_transport::ImageTransport(nh);

  // set up callbacks
  for (size_t i = 0; i < vioParameters_.nCameraSystem.numCameras(); ++i) {
    imageSubscribers_[i] = imgTransport_->subscribe(
        "/camera" + std::to_string(i),
        30 * vioParameters_.nCameraSystem.numCameras(),
        std::bind(&Subscriber::imageCallback, this, std::placeholders::_1, i));
  }

  subImu_ = nh_->subscribe("/imu", 1000, &Subscriber::imuCallback, this);
}

void Subscriber::imageCallback(const sensor_msgs::ImageConstPtr& msg,/*
 const sensor_msgs::CameraInfoConstPtr& info,*/
                               unsigned int cameraIndex)
{
  const cv::Mat raw(msg->height, msg->width, CV_8UC1,
                    const_cast<uint8_t*>(&msg->data[0]), msg->step);

  cv::Mat filtered;
  if (vioParameters_.optimization.useMedianFilter) {
    cv::medianBlur(raw, filtered, 3);
  } else {
    filtered = raw.clone();
  }

  // adapt timestamp
  okvis::Time t(msg->header.stamp.sec, msg->header.stamp.nsec);
  t -= okvis::Duration(vioParameters_.sensors_information.imageDelay);

  if (!vioInterface_->addImage(t, cameraIndex, filtered))
    LOG(WARNING) << "Frame delayed at time "<<t;

  // TODO: pass the keypoints...
}

void Subscriber::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  vioInterface_->addImuMeasurement(
      okvis::Time(msg->header.stamp.sec, msg->header.stamp.nsec),
      Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y,
                      msg->linear_acceleration.z),
      Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y,
                      msg->angular_velocity.z));
}

#ifdef HAVE_LIBVISENSOR
void Subscriber::initialiseDriverCallbacks()
{
  // mostly copied from https://github.com/ethz-asl/visensor_node_devel
  if (sensor_ == nullptr) {
    sensor_ = std::unique_ptr<visensor::ViSensorDriver>(
        new visensor::ViSensorDriver());

    try {
      // use autodiscovery to find sensor. TODO: specify IP in config?
      sensor_->init();
    } catch (Exception const &ex) {
      LOG(ERROR) << ex.what();
      exit(1);
    }
  }

  try {
    sensor_->setCameraCallback(
        std::bind(&Subscriber::directFrameCallback, this, std::placeholders::_1,
                  std::placeholders::_2));
    sensor_->setImuCallback(
        std::bind(&Subscriber::directImuCallback, this, std::placeholders::_1,
                  std::placeholders::_2));
    sensor_->setFramesCornersCallback(
        std::bind(&Subscriber::directFrameCornerCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    std::vector<visensor::SensorId::SensorId> listOfCameraIds =
        sensor_->getListOfCameraIDs();

    for (auto it = listOfCameraIds.begin(); it != listOfCameraIds.end(); ++it) {
      // TODO(burrimi): move this to some sort of configuration file.
      const int is_flipped = 1;
      const int slot_number = 0; // 0 is factory calibration
      sensor_->setCameraCalibrationToUse(*it, slot_number, is_flipped,
                                         visensor::ViCameraLensModel::LensModelTypes::UNKNOWN, // TODO(burrimi): which one to select?
                                         visensor::ViCameraProjectionModel::ProjectionModelTypes::PINHOLE);
    }
  } catch (Exception const &ex) {
    LOG(ERROR) << ex.what();
  }
}
#endif

#ifdef HAVE_LIBVISENSOR
void Subscriber::startSensors(const std::vector<unsigned int>& camRate,
                              const unsigned int imuRate)
{
  // mostly copied from https://github.com/ethz-asl/visensor_node_devel
  OKVIS_ASSERT_TRUE_DBG(Exception,sensor_ != nullptr,"Sensor pointer not yet initialised.");

  std::vector<visensor::SensorId::SensorId> listOfCameraIds = sensor_
      ->getListOfCameraIDs();

  OKVIS_ASSERT_TRUE_DBG(Exception,listOfCameraIds.size() == camRate.size(),"Number of cameras don't match up.");

  for (uint i = 0; i < listOfCameraIds.size(); i++) {
    if (camRate[i] > 0)
      sensor_->startSensor(listOfCameraIds[i], camRate[i]);
  }

  sensor_->startAllCorners();
  sensor_->startSensor(visensor::SensorId::IMU0, imuRate);
  /*if (sensor_->isSensorPresent(visensor::SensorId::LED_FLASHER0))
    sensor_->startSensor(visensor::SensorId::LED_FLASHER0);*/ // apparently experimental...
}
#endif

#ifdef HAVE_LIBVISENSOR
void Subscriber::directImuCallback(
    boost::shared_ptr<visensor::ViImuMsg> imu_ptr, visensor::ViErrorCode error)
{
  if (error == visensor::ViErrorCodes::MEASUREMENT_DROPPED) {
    LOG(WARNING) << "dropped imu measurement on sensor "
                 << imu_ptr->imu_id
                 << " (check network bandwidth/sensor rate)";
    return;
  }

  okvis::Time timestamp;
  timestamp.fromNSec(imu_ptr->timestamp);

  vioInterface_->addImuMeasurement(
      timestamp,
      Eigen::Vector3d(imu_ptr->acc[0], imu_ptr->acc[1], imu_ptr->acc[2]),
      Eigen::Vector3d(imu_ptr->gyro[0], imu_ptr->gyro[1], imu_ptr->gyro[2]));
}
#endif

#ifdef HAVE_LIBVISENSOR
void Subscriber::directFrameCallback(visensor::ViFrame::Ptr frame_ptr,
                                     visensor::ViErrorCode error)
{
  if (error == visensor::ViErrorCodes::MEASUREMENT_DROPPED) {
    LOG(WARNING) << "dropped camera image on sensor "
                 << frame_ptr->camera_id
                 << " (check network bandwidth/sensor rate)";
    return;
  }

  int image_height = frame_ptr->height;
  int image_width = frame_ptr->width;

  okvis::Time timestamp;
  timestamp.fromNSec(frame_ptr->timestamp);

  // check if transmission is delayed
  const double frame_delay = (okvis::Time::now() - timestamp).toSec();
  if (frame_delay > THRESHOLD_DATA_DELAY_WARNING)
    LOG(WARNING) << "Data arrived later than expected [ms]: " << frame_delay * 1000.0;

  cv::Mat raw;
  if (frame_ptr->image_type == visensor::MONO8) {
    raw = cv::Mat(image_height, image_width, CV_8UC1);
    memcpy(raw.data, frame_ptr->getImageRawPtr(), image_width * image_height);
  } else if (frame_ptr->image_type == visensor::MONO16) {
    raw = cv::Mat(image_height, image_width, CV_16UC1);
    memcpy(raw.data, frame_ptr->getImageRawPtr(),
           (image_width) * image_height * 2);
  } else {
    LOG(WARNING) << "[VI_SENSOR] - unknown image type!";
    return;
  }

  cv::Mat filtered;
  if (vioParameters_.optimization.useMedianFilter) {
    cv::medianBlur(raw, filtered, 3);
  } else {
    filtered = raw.clone();
  }

  // adapt timestamp
  timestamp -= okvis::Duration(vioParameters_.sensors_information.imageDelay);

  if (!vioInterface_->addImage(timestamp, frame_ptr->camera_id, filtered))
    LOG(WARNING) << "Frame delayed at time " << timestamp;
}
#endif

#ifdef HAVE_LIBVISENSOR
void Subscriber::directFrameCornerCallback(visensor::ViFrame::Ptr /*frame_ptr*/,
                                           visensor::ViCorner::Ptr /*corners_ptr*/)
{
  LOG(INFO) << "directframecornercallback";
}
#endif

} // namespace okvis
