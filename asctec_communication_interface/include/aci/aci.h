/*
 * Copyright (c) 2014, Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#ifndef ACI_H_
#define ACI_H_

#include <thread>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <map>
#include <vector>
#include <list>

#include <aci/implementation/asctecCommIntf.h>
#include <aci/raw_buffer.h>
#include <aci/timesync_base.h>
#include <aci/variable.h>

namespace aci {

class SubscriptionObj;
typedef std::shared_ptr<SubscriptionObj> Subscription;

typedef std::function<void()> VarPacketCallback;
typedef std::list<VarPacketCallback> VarPacketSubscriptionList;
typedef std::list<VarPacketCallback>::iterator VarPacketSubscription;

enum {
  VAR_PACKET_FAST = 0,
  VAR_PACKET_MEDIUM = 1,
  VAR_PACKET_SLOW = 2
};

enum class FcuType {
  AutoPilot,
  Trinity,
  Unknown
};

enum class MavType {
  Firefly,
  Falcon,
  Hummingbird,
  Neo_6_9,
  Neo_6_11,
  Pelican,
  Unknown
};

std::string mavTypeToString(const MavType& type);
std::string flightModeToStringTrinity(uint32_t flight_mode);

static constexpr double kDefaultTrinityRate = 500.0;
static constexpr double kDefaultAutoPilotRate = 1000.0;

class Aci {
 public:

  static Aci& instance() {
    static Aci instance;
    return instance;
  }
  ~Aci();

  void shutdown();

  template<class VariableType_>
  Variable<VariableType_> registerVariable(int packet_id, int variable_id);

  template<class VariableType_>
  Variable<VariableType_> registerCommand(int packet_id, int command_id);

  bool init(RawBufferPtr raw_buffer);

  /**
   * \brief Same as init(RawBufferPtr raw_buffer), but allows to set time synchronization components to reduce
   *        waiting time;
   */
  bool init(RawBufferPtr raw_buffer, std::shared_ptr<TimeSyncBase> time_synchronizer,
            std::shared_ptr<LocalClockBase> local_clock);

  Subscription registerVariableCallback(int packet_id, const VarPacketCallback& callback);

  template<class T>
  Subscription registerVariableCallback(int packet_id, void (T::*cb_func)(), T* obj) {
    return registerVariableCallback(packet_id, std::bind(cb_func, obj));
  }

  void setVariablePacketRate(int packet_id, double rate);
  void updateVariableConfiguration(int packet_id);

  void updateCommandConfiguration(int packet_id, bool with_acknowledgement);

  void unregisterVariableCallback(int packet_id, const VarPacketSubscription& sub);

  int getNextFreeCommandPacketId() const;

  void resetCommandPacket(int packet_id);

  void sendCommandPacketToDevice(int packet_id);

  template<class VariableType_, class Representation, class Duration>
  bool requestSingleVariable(Variable<VariableType_>& variable, int id,
                             const std::chrono::duration<Representation, Duration>& timeout);

  bool setTimeSynchronizer(std::shared_ptr<TimeSyncBase> time_synchronizer);
  bool setLocalClock(std::shared_ptr<LocalClockBase> local_clock);
  bool setTimeSynchronizerAndLocalClock(std::shared_ptr<TimeSyncBase> time_synchronizer,
                                        std::shared_ptr<LocalClockBase> local_clock);

  bool deviceTimeToLocalTime(double device_time, double* local_time);
  bool localTimeToDeviceTime(double local_time, double* device_time);

  MavType getMavType() const {
    return mav_type_;
  }

  FcuType getFcuType() const {
    return fcu_type_;
  }

  bool isInitialized() const {
    return initialized_;
  }

  void printVersionInfo() const;
  static void printVariableInfo();
  static void printVariableDefines();
  static void printCommandInfo();
  static void printCommandDefines();

 private:
  typedef std::unique_lock<std::recursive_mutex> UniqueLock;
  typedef std::map<unsigned short, VariableBase::Ptr> VariablePacket;

  static constexpr int kEngineRate = 100;
  static constexpr int kHeartbeatrate = 10;

  static constexpr int kTimesyncWaitTimeMs = 500;

  Aci();
  Aci(const Aci&);
  Aci& operator=(const Aci&);

  void startCommunicationThreads();
  void stopCommunicationThreads();
  void startTimeSynchronizationThread();
  void stopTimeSynchronizationThread();

  void engineThread();
  void rxThread();
  void timeSynchronizationThread();

  static void writeBuffer(void* data, unsigned short cnt) {
    Aci::instance().raw_buffer_->writeBuffer(reinterpret_cast<uint8_t*>(data), cnt);
  }

  bool getDeviceInfo();
  bool getDeviceVariableList();
  bool getDeviceCommandList();
  bool getDeviceParameterList();

  static void versionInfoCallback(struct ACI_INFO aciInfo);
  static void variableListUpdateFinished(void);
  static void commandListUpdateFinished(void);
  static void parameterListUpdateFinished(void);
  static void singleVariableReceived(unsigned short id, void *data, unsigned char varType);

  static void variablePacketUpdated(unsigned char id);

  bool detectFcuAndMavType(FcuType* fcu_type, MavType* mav_type);

  RawBufferPtr raw_buffer_;

  bool rx_thread_running_;
  bool engine_thread_running_;
  bool time_synchronization_thread_running_;

  bool have_device_info_;
  bool have_device_variables_;
  bool have_device_commands_;
  bool have_device_parameters_;

  bool initialized_;

  std::vector<VariablePacket> variable_packets_;
  std::vector<VarPacketSubscriptionList> variable_subscriptions_;

  std::vector<VariablePacket> command_packets_;
  std::list<int> free_command_packet_ids_;

  std::thread engine_thread_;
  std::thread rx_thread_;
  std::thread time_synchronization_thread_;
  std::recursive_mutex mutex_;
  std::recursive_mutex time_synchronization_mutex_;

  struct ACI_INFO device_version_info_;

  std::condition_variable_any single_request_condition_; ///< Condition variable that wakes up the function that sent the single request.
  uint8_t single_request_buffer_[kAsctecMaximumVariableSize]; ///< Temporary buffer for incoming single request variables. Size equals max. variable size as defined in aci (6 bits).
  uint16_t single_request_returned_id_; ///< Id of the last variable that was returned by the device.

  std::shared_ptr<TimeSyncBase> time_synchronizer_;
  std::shared_ptr<LocalClockBase> local_clock_;

  FcuType fcu_type_;
  MavType mav_type_;
};

class SubscriptionObj {
 public:
  SubscriptionObj(int packet_id, const VarPacketSubscription& it)
      : packet_id_(packet_id),
        subscription_(it) {
  }

  ~SubscriptionObj() {
    Aci::instance().unregisterVariableCallback(packet_id_, subscription_);
  }
 private:
  SubscriptionObj(const SubscriptionObj&);
  SubscriptionObj& operator=(const SubscriptionObj&);

  int packet_id_;
  VarPacketSubscription subscription_;
};

inline Aci& instance(){
  return Aci::instance();
}

} // end namespace aci

#include <aci/implementation/aci_impl.h>

#endif /* ACI_H_ */
