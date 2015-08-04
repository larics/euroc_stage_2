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

#ifndef TIMESYNC_BASE_H_
#define TIMESYNC_BASE_H_

namespace aci {

class TimeSyncBase {
 public:
  TimeSyncBase();
  virtual ~TimeSyncBase() {}
  void reset();
  bool update();
  bool addRoundTripMeasurement(double local_time_sent, double device_time_received, double device_time_sent,
                               double local_time_received);
  bool deviceTimeToLocalTime(double device_time, double* local_time);
  bool localTimeToDeviceTime(double local_time, double* device_time);

  /// Returns the initialization status of the time-synchronizer from 0...100%.
  virtual int getInitializationStatus() const = 0;

 protected:
  virtual void resetImpl() = 0;
  virtual bool updateImpl() = 0;

  virtual bool addRoundTripMeasurementImpl(double local_time_sent, double device_time_received,
                                           double device_time_sent, double local_time_received) = 0;
  virtual bool deviceTimeToLocalTimeImpl(double device_time, double* local_time) = 0;
  virtual bool localTimeToDeviceTimeImpl(double local_time, double* device_time) = 0;

 private:
  double last_conversion_to_local_;
  double last_conversion_to_device_;
  double last_local_time_;
  double last_device_time_;
};

class LocalClockBase {
 public:
  virtual ~LocalClockBase() {}

  virtual double now();
};

} // end namespace aci

#endif /* TIMESYNC_BASE_H_ */
