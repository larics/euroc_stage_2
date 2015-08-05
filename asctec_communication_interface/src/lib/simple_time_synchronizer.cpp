/*
 * Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <Eigen/Dense>

#include <aci/macros.h>
#include <aci/simple_time_synchronizer.h>

namespace aci {

SimpleTimeSynchronizer::SimpleTimeSynchronizer()
    : skew_(1),
      offset_(0),
      sigma_delay_(kDefaultSigmaDelay),
      buffer_size_(kDefaultBufferSize),
      initialization_status_(0) {
}

SimpleTimeSynchronizer::~SimpleTimeSynchronizer() {
}

void SimpleTimeSynchronizer::resetImpl() {
  measurement_buffer_.clear();
  skew_ = 1;
  offset_ = 0;
}

bool SimpleTimeSynchronizer::updateImpl() {
  return solve(&skew_, &offset_);
}

bool SimpleTimeSynchronizer::setSigmaDelay(double sigma_delay) {
  if (sigma_delay < 0) {
    ACI_ERROR_STREAM("sigma has to be > 0");
    return false;
  }

  sigma_delay_ = sigma_delay;
  return true;
}

void SimpleTimeSynchronizer::setBufferSize(size_t buffer_size) {
  buffer_size_ = buffer_size;
}

void SimpleTimeSynchronizer::getParameters(double* skew, double* offset) {
  if (skew != nullptr && offset != nullptr) {
    *skew = skew_;
    *offset = offset_;
  }
  else {
    ACI_ERROR_STREAM("don't give me nullptrs!!!");
  }
}

int SimpleTimeSynchronizer::getInitializationStatus () const {
  return initialization_status_;
}

bool SimpleTimeSynchronizer::addRoundTripMeasurementImpl(double local_time_sent, double device_time_received,
                                                         double device_time_sent, double local_time_received) {
  Measurement m(local_time_sent, device_time_received, local_time_received);
  if (checkMeasurement(m)) {
    measurement_buffer_.push_back(m);
    while (measurement_buffer_.size() > buffer_size_) {
      measurement_buffer_.pop_front();
    }
    initialization_status_ = (measurement_buffer_.size() * 100) / kDefaultMinimumMeasurements;
    if (initialization_status_ > 100) {
      initialization_status_ = 100;
    }
  }
  return true;
}

bool SimpleTimeSynchronizer::deviceTimeToLocalTimeImpl(double device_time, double* local_time) {
  *local_time = device_time * skew_ + offset_;
  return true;
}

bool SimpleTimeSynchronizer::localTimeToDeviceTimeImpl(double local_time, double* device_time) {
  *device_time = (local_time - offset_) / skew_;
  return true;
}

bool SimpleTimeSynchronizer::solve(double* skew, double* offset) {
  if (skew == nullptr || offset == nullptr)
    return false;

  const size_t n_measurements = measurement_buffer_.size();

  // Solve: 2*device_time*skew + 2*offset = local_time_sent + local_time_received.

  Eigen::VectorXd b(n_measurements);
  Eigen::MatrixX2d A;
  A.resize( n_measurements, Eigen::NoChange);

  // Only take measurements into account where the roundtrip time is less than 3 sigma_delay.
  int count = 0;

  for (const Measurement& m : measurement_buffer_) {
    if (checkMeasurement(m)) {
      b(count) = m.local_time_received + m.local_time_sent;
      A(count, 0) = 2.0 * m.device_time;
      A(count, 1) = 2.0;
      ++count;
    }
  }

  if(count < 2){
    ACI_ERROR_STREAM("solving for skew and offset needs at least 2 measurements, but have only " << count);
    return false;
  }

  Eigen::Vector2d result = A.block(0,0,count, 2).jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b.head(count));

  skew_ = result(0);
  offset_ = result(1);

  ACI_DEBUG_STREAM("simple time sync: skew="<<skew_ << " offset=" << offset_ << " n_meas=" <<
                   measurement_buffer_.size() << "actually used meas=" << count);

  return true;
}

bool SimpleTimeSynchronizer::checkMeasurement(const Measurement& measurement) const {
  constexpr double sigma3 = 3 * kDefaultSigmaDelay;
  return (measurement.local_time_received - measurement.local_time_sent) < sigma3;
}

} /* namespace aci */
