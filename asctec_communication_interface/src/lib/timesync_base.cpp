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

#include <chrono>

#include <aci/macros.h>
#include <aci/timesync_base.h>

namespace aci {

TimeSyncBase::TimeSyncBase()
    : last_conversion_to_local_(0),
      last_conversion_to_device_(0),
      last_local_time_(0),
      last_device_time_(0) {
}

void TimeSyncBase::reset() {
  resetImpl();
}

bool TimeSyncBase::update() {
  return updateImpl();
}

bool TimeSyncBase::addRoundTripMeasurement(double local_time_sent, double device_time_received,
                                           double device_time_sent, double local_time_received) {

  // In case this executed in a hardware in the loop simulation, it can happen that device_time_sent is equal
  // to device_time_received.
  constexpr double epsilon = 1.0e-9;
  if(std::abs(local_time_sent - local_time_received) < epsilon) {
    ACI_INFO_STREAM("local_time_sent is appox. local_time_received --> is this run in simulation?");
    return false;
  }

  ACI_DEBUG_STREAM("got roundtrip measurement:"
                   "\n    local_time_sent=" << std::setprecision(18) << local_time_sent <<
                   "\n    device_time_received=" << std::setprecision(18) << device_time_received <<
                   "\n    device_time_sent=" << std::setprecision(18) << device_time_sent <<
                   "\n    local_time_received=" << std::setprecision(18) << local_time_received);

  return addRoundTripMeasurementImpl(local_time_sent, device_time_received, device_time_sent,
                                     local_time_received);
}

bool TimeSyncBase::deviceTimeToLocalTime(double device_time, double* local_time) {

  if (local_time == nullptr) {
    ACI_ERROR_STREAM("Nullptr in deviceTimeToLocalTime!");
    return false;
  }

  if (device_time < 0) {
    ACI_ERROR_STREAM("device time < 0, is this intended");
    return false;
  }

  // Try to use cached time to ensure that conversions are consistent.
  if (last_device_time_ == device_time) {
    *local_time = last_conversion_to_local_;
  }
  else {
    if (!deviceTimeToLocalTimeImpl(device_time, local_time))
      return false;
  }

//  ACI_INFO_STREAM("Device_time:" << device_time <<
//                  " last_conversion_to_local_: " << last_conversion_to_local_ <<
//                  " new local_time: " << *local_time);

  if (*local_time < last_conversion_to_local_) {
    ACI_ERROR_STREAM("time went backwards in conversion to local time! Device_time:" << device_time <<
                     " last_conversion_to_local_: " << last_conversion_to_local_ <<
                     " new local_time: " << *local_time);
    return false;
  }

  last_device_time_ = device_time;
  last_conversion_to_local_ = *local_time;
  return true;
}

bool TimeSyncBase::localTimeToDeviceTime(double local_time, double* device_time) {
  if (device_time == nullptr) {
    ACI_ERROR_STREAM("Nullptr in localTimeToDeviceTime!");
    return false;
  }

  if (local_time < 0) {
    ACI_ERROR_STREAM("local time < 0, is this intended");
    return false;
  }

  // Try to use cached time to ensure that conversions are consistent.
  if (last_local_time_ == local_time) {
    return last_local_time_;
  }
  else {
    if (!localTimeToDeviceTimeImpl(local_time, device_time))
      return false;
  }

  if (*device_time < last_conversion_to_device_) {
    ACI_ERROR_STREAM("time went backwards in conversion to device time! Local time:" << local_time <<
                     "last dev. time: " << last_conversion_to_device_ << " new dev. time: " << *device_time);
    return false;
  }

  last_local_time_ = local_time;
  last_conversion_to_device_ = *device_time;
  return true;
}

double LocalClockBase::now() {
  typedef std::chrono::duration<double> DurationDouble;
  auto now = std::chrono::system_clock::now();
  return std::chrono::duration_cast<DurationDouble>(now.time_since_epoch()).count();
}

} // end namespace aci
