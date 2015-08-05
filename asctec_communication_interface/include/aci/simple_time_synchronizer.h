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

#ifndef SIMPLE_TIME_SYNCHRONIZER_H_
#define SIMPLE_TIME_SYNCHRONIZER_H_

#include <deque>
#include <stdlib.h>

#include <aci/timesync_base.h>

namespace aci {

class SimpleTimeSynchronizer : public TimeSyncBase {
 public:
  SimpleTimeSynchronizer();
  virtual ~SimpleTimeSynchronizer();

  bool setSigmaDelay(double sigma_delay);
  void setBufferSize(size_t buffer_size);
  void getParameters(double* skew, double* offset);
  virtual int getInitializationStatus () const;

 protected:
  virtual void resetImpl();
  virtual bool updateImpl();
  virtual bool addRoundTripMeasurementImpl(double local_time_sent, double device_time_received,
                                           double device_time_sent, double local_time_received);
  virtual bool deviceTimeToLocalTimeImpl(double device_time, double* local_time);
  virtual bool localTimeToDeviceTimeImpl(double device_time, double* local_time);

 private:
  static constexpr size_t kDefaultBufferSize = 100;
  static constexpr double kDefaultSigmaDelay = 0.005;
  static constexpr int kDefaultMinimumMeasurements = 10;

  bool solve(double* skew, double* offset);

  class Measurement{
   public:
    Measurement()
        : local_time_sent(0),
          device_time(0),
          local_time_received(0) {
    }

    Measurement(double _local_time_sent, double _device_time, double _local_time_received)
        : local_time_sent(_local_time_sent),
          device_time(_device_time),
          local_time_received(_local_time_received) {
    }

    double local_time_sent;
    double device_time;
    double local_time_received;
  };

  bool checkMeasurement(const Measurement& measurement) const;

  double skew_;
  double offset_;
  double sigma_delay_;
  size_t buffer_size_;
  int initialization_status_;

  std::deque<Measurement> measurement_buffer_;
};

} /* namespace aci */

#endif /* SIMPLE_TIME_SYNCHRONIZER_H_ */
