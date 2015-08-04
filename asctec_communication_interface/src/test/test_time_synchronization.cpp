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
#include <random>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <aci/simple_time_synchronizer.h>

const int n_repeated_tests = 100;

void generateTestData(size_t N, double skew, double offset, double interval, double sigma_delay,
                      double spike_ratio, double spike_amplitude, size_t seed,
                      std::vector<double>* local_time_sent, std::vector<double>* device_time_received,
                      std::vector<double>* local_time_received);

bool testSynchronizer(size_t N, double skew, double offset, double interval, double sigma_delay,
                      double spike_ratio, double spike_amplitude, double max_relative_skew_error,
                      double max_absolute_offset_error);

// Compute the mean of the absolute normal distribution. Only for zero mean.
// http://en.wikipedia.org/wiki/Folded_normal_distribution
double meanOfAbsNormalDistribution(double stddev) {
  return stddev * sqrt(2.0 / M_PI);
}

// Compute the new standard deviation of the absolute normal distribution. Only for original distribution with zero mean.
// http://en.wikipedia.org/wiki/Folded_normal_distribution
double stddevOfAbsNormalDistribution(double stddev) {
  const double mean = meanOfAbsNormalDistribution(stddev);
  // stddev_new = sqrt(stddev * stddev - mean * mean)
  return sqrt(stddev * stddev * (1.0 - 2.0 / M_PI));
}

void generateTestData(size_t N, double skew, double offset, double interval, double sigma_delay,
                      double spike_ratio, double spike_amplitude, size_t seed,
                      std::vector<double>* local_time_sent, std::vector<double>* device_time_received,
                      std::vector<double>* local_time_received) {

  EXPECT_GE(N, 2);
  EXPECT_GE(skew, 0.0);
  EXPECT_GT(interval, 0.0);
  EXPECT_GT(sigma_delay, 0.0);
  EXPECT_GE(spike_ratio, 0.0);
  EXPECT_LE(spike_ratio, 1.0);
  EXPECT_GE(spike_amplitude, 0.0);

  std::mt19937 generator(seed);
  std::uniform_real_distribution<double> uniform_distribution(0.0, spike_amplitude);
  std::normal_distribution<double> normal_distribution(0.0, sigma_delay);

  local_time_sent->resize(N, 0.0);
  device_time_received->resize(N, 0.0);
  local_time_received->resize(N, 0.0);

  // local_time = device_time * skew + offset
  // local_time_sent --> noise_tx --> spike_tx --> device_time_received --> noise_rx --> spike_rx --> local_time_received
  for (size_t i = 0; i < N; ++i) {

    double noise_tx = std::abs(normal_distribution(generator));
    double noise_rx = std::abs(normal_distribution(generator));

    double spike_tx = uniform_distribution(generator);
    if (spike_tx < (1.0 - spike_ratio))
      spike_tx = 0;
    else
      spike_tx *= spike_amplitude;

    double spike_rx = uniform_distribution(generator);
    if (spike_rx < (1.0 - spike_ratio))
      spike_rx = 0;
    else
      spike_rx *= spike_amplitude;

    (*local_time_sent)[i] = static_cast<double>(i) * interval + offset;
    (*device_time_received)[i] = ((*local_time_sent)[i] + noise_tx + spike_tx - offset) / skew;  // need to transform to device time
    (*local_time_received)[i] = (*device_time_received)[i] * skew + offset + noise_rx + spike_rx;  // transform device time back, add noise / spikes
  }
}

bool testSynchronizer(size_t N, double skew, double offset, double interval, double sigma_delay,
                      double spike_ratio, double spike_amplitude, double max_relative_skew_error,
                      double max_absolute_offset_error) {

  std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();

  for (size_t n_test = 0; n_test < n_repeated_tests; ++n_test) {
    const size_t seed = n_test;

    std::vector<double> local_time_sent, device_time, local_time_received;
    generateTestData(N, skew, offset, interval, sigma_delay, spike_ratio, spike_amplitude, seed,
                     &local_time_sent, &device_time, &local_time_received);

    aci::SimpleTimeSynchronizer time_sync;
    time_sync.setSigmaDelay(sigma_delay);
    time_sync.setBufferSize(N);

    for (size_t i = 0; i < N; ++i) {
      EXPECT_TRUE(
          time_sync.addRoundTripMeasurement(local_time_sent[i], device_time[i], device_time[i],
                                            local_time_received[i]));
    }

    EXPECT_TRUE(time_sync.update());

    double estimated_skew, estimated_offset;
    time_sync.getParameters(&estimated_skew, &estimated_offset);

    double relative_skew_error = std::abs(1.0 - estimated_skew / skew);

    EXPECT_NEAR(relative_skew_error, 0, max_relative_skew_error) << "true skew vs. estimated: " << skew
                                                                 << " / " << estimated_skew;
    EXPECT_NEAR(estimated_offset, offset, max_absolute_offset_error);
  }

  std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
  std::cout
      << "[   info   ] average test duration "
      << std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count() / n_repeated_tests
      << " us" << std::endl;

  return true;
}

TEST(ACI, SimpleTimeSync_small_buffer_high_noise) {
  size_t N = 10;
  const double skew = 1.0001;
  const double offset = 1.0;
  const double sigma_delay = 0.01;
  const double interval = 0.5;
  const double spike_ratio = 0.1;
  const double spike_amplitude = 0.1;
  // We allow higher errors here, since this usually just happens in the initialization phase.
  const double max_relative_skew_error = 0.05;
  const double max_absolute_offset_error = stddevOfAbsNormalDistribution(sigma_delay) * 3;
  testSynchronizer(N, skew, offset, interval, sigma_delay, spike_ratio, spike_amplitude,
                   max_relative_skew_error, max_absolute_offset_error);
}

TEST(ACI, SimpleTimeSync_small_buffer_low_noise) {
  size_t N = 10;
  const double skew = 1.0001;
  const double offset = 1.0;
  const double sigma_delay = 0.005;
  const double interval = 0.5;
  const double spike_ratio = 0.05;
  const double spike_amplitude = 0.05;
  const double max_relative_skew_error = 0.005;
  const double max_absolute_offset_error = stddevOfAbsNormalDistribution(sigma_delay);
  testSynchronizer(N, skew, offset, interval, sigma_delay, spike_ratio, spike_amplitude,
                   max_relative_skew_error, max_absolute_offset_error);
}

TEST(ACI, SimpleTimeSync_large_buffer_high_noise) {
  size_t N = 100;
  const double skew = 1.0001;
  const double offset = 1.0;
  const double sigma_delay = 0.01;
  const double interval = 0.5;
  const double spike_ratio = 0.1;
  const double spike_amplitude = 0.1;
  const double max_relative_skew_error = 0.005;
  const double max_absolute_offset_error = stddevOfAbsNormalDistribution(sigma_delay);
  testSynchronizer(N, skew, offset, interval, sigma_delay, spike_ratio, spike_amplitude,
                   max_relative_skew_error, max_absolute_offset_error);
}

TEST(ACI, SimpleTimeSync_large_buffer_low_noise) {
  size_t N = 100;
  const double skew = 1.0001;
  const double offset = 1.0;
  const double sigma_delay = 0.005;
  const double interval = 0.5;
  const double spike_ratio = 0.05;
  const double spike_amplitude = 0.05;
  const double max_relative_skew_error = 0.005;
  const double max_absolute_offset_error = stddevOfAbsNormalDistribution(sigma_delay);
  testSynchronizer(N, skew, offset, interval, sigma_delay, spike_ratio, spike_amplitude,
                   max_relative_skew_error, max_absolute_offset_error);
}

TEST(ACI, SimpleTimeSync_autopilot_large_buffer_high_noise) {
  size_t N = 100;
  const double skew = 1.0e-3;  // autopilot has a ms counter
  const double offset = (2015.0 - 1970.0) * 365.0 * 24.0 * 60.0 * 60.0;  // autopilot starts counting from 0
  const double sigma_delay = 0.005;
  const double interval = 0.5;
  const double spike_ratio = 0.1;
  const double spike_amplitude = 0.1;
  const double max_relative_skew_error = 0.005;
  const double max_absolute_offset_error = stddevOfAbsNormalDistribution(sigma_delay);
  testSynchronizer(N, skew, offset, interval, sigma_delay, spike_ratio, spike_amplitude,
                   max_relative_skew_error, max_absolute_offset_error);
}

TEST(ACI, SimpleTimeSync_autopilot_small_buffer_high_noise) {
  size_t N = 1000;
  const double skew = 1.0e-3;  // autopilot has a ms counter
  const double offset = (2015.0 - 1970.0) * 365.0 * 24.0 * 60.0 * 60.0;  // autopilot starts counting from 0
  const double sigma_delay = 0.005;
  const double interval = 0.5;
  const double spike_ratio = 0.1;
  const double spike_amplitude = 0.1;
  // We allow higher errors here, since this usually just happens in the initialization phase.
  const double max_relative_skew_error = 0.05;
  const double max_absolute_offset_error = stddevOfAbsNormalDistribution(sigma_delay) * 3;
  testSynchronizer(N, skew, offset, interval, sigma_delay, spike_ratio, spike_amplitude,
                   max_relative_skew_error, max_absolute_offset_error);
}

TEST(ACI, SimpleTimeSync_does_time_go_backwards) {
  const size_t seed = 12345;
  size_t N = 1000;
  const double interval = 0.5; // Together with N, this simulates 5000s runtime.
  size_t buffer_size = 100;
  const double skew = 1.0e-3;  // autopilot has a ms counter
  const double offset = (2015.0 - 1970.0) * 365.0 * 24.0 * 60.0 * 60.0;  // autopilot starts counting from 0
  const double sigma_delay = 0.005;
  const double spike_ratio = 0.1;
  const double spike_amplitude = 0.1;
  const double fastest_packet_period = 0.01; // 100 Hz

  std::vector<double> local_time_sent, device_time_received, local_time_received;
  generateTestData(N, skew, offset, interval, sigma_delay, spike_ratio, spike_amplitude, seed,
                   &local_time_sent, &device_time_received, &local_time_received);

  aci::SimpleTimeSynchronizer time_sync;
  time_sync.setSigmaDelay(sigma_delay);
  time_sync.setBufferSize(buffer_size);

  double last_local_time = 0;
  double last_device_time = 0;
  for (size_t i = 0; i < N; ++i) {
    EXPECT_TRUE(
        time_sync.addRoundTripMeasurement(local_time_sent[i], device_time_received[i], device_time_received[i],
                                          local_time_received[i]));
    if (i > 10) {
      EXPECT_TRUE(time_sync.update());

      // TODO(acmarkus): this is tricky business: if we start at t=0, it can happen that we convert almost the
      // same time twice. If the parameters of the mapping function changed, it can happen that time goes backwards.
      for (double t = fastest_packet_period; t < interval; t += fastest_packet_period) {
        double local_time, device_time;
        time_sync.deviceTimeToLocalTime(device_time_received[i] + t / skew, &local_time);
        time_sync.localTimeToDeviceTime(local_time_sent[i] + t, &device_time);
        EXPECT_GT(local_time, last_local_time) << "difference: " << local_time - last_local_time << " t:" << t
                                               << " i: " << i;
        EXPECT_GT(device_time, last_device_time) << "difference: " << device_time - last_device_time << " t:"
                                                 << t << " i: " << i;
        last_local_time = local_time;
        last_device_time = device_time;
      }
    }
  }

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
