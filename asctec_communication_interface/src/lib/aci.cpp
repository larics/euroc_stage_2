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

#include <chrono>

#include <aci/aci.h>
#include <aci/implementation/asctecCommIntf.h>
#include <aci/macros.h>
#include <aci/simple_time_synchronizer.h>
#include <aci/variable_defines_autopilot.h>
#include <aci/variable_defines_trinity.h>

#include <iostream>
#include <iomanip>

namespace aci {

constexpr int Aci::kEngineRate;
constexpr int Aci::kHeartbeatrate;
constexpr int Aci::kTimesyncWaitTimeMs;

Aci::Aci()
    : rx_thread_running_(false),
      engine_thread_running_(false),
      time_synchronization_thread_running_(false),
      have_device_info_(false),
      have_device_variables_(false),
      have_device_commands_(false),
      have_device_parameters_(false),
      initialized_(false),
      single_request_returned_id_(0),
      fcu_type_(FcuType::Unknown),
      mav_type_(MavType::Unknown) {

  variable_packets_.resize(MAX_VAR_PACKETS);
  variable_subscriptions_.resize(MAX_VAR_PACKETS);

  // Keep track of IDs free command packets. All are free at the beginning.
  command_packets_.resize(MAX_VAR_PACKETS);
  for (int i = 0; i < MAX_VAR_PACKETS; ++i)
    free_command_packet_ids_.push_back(i);
}

void Aci::startCommunicationThreads() {
  rx_thread_running_ = true;
  engine_thread_running_ = true;
  rx_thread_ = std::thread(std::bind(&Aci::rxThread, this));
  engine_thread_ = std::thread(std::bind(&Aci::engineThread, this));
}

void Aci::stopCommunicationThreads() {
  rx_thread_running_ = false;
  engine_thread_running_ = false;

  if (engine_thread_.joinable())
    engine_thread_.join();
  if (rx_thread_.joinable())
    rx_thread_.join();
}

void Aci::startTimeSynchronizationThread() {
  time_synchronization_thread_running_ = true;
  time_synchronization_thread_ = std::thread(std::bind(&Aci::timeSynchronizationThread, this));
}

void Aci::stopTimeSynchronizationThread() {
  time_synchronization_thread_running_ = false;
  if (time_synchronization_thread_.joinable())
    time_synchronization_thread_.join();
}

bool Aci::init(RawBufferPtr raw_buffer) {
  stopCommunicationThreads();
  raw_buffer_ = raw_buffer;

  aciInit();
  aciSetEngineRate(kEngineRate, kHeartbeatrate);

  aciSetSendDataCallback(&Aci::writeBuffer);
  aciVarPacketReceivedCallback(&Aci::variablePacketUpdated);

  aciInfoPacketReceivedCallback(&Aci::versionInfoCallback);
  aciSetVarListUpdateFinishedCallback(&Aci::variableListUpdateFinished);
  aciSetCmdListUpdateFinishedCallback(&Aci::commandListUpdateFinished);
  aciSetParamListUpdateFinishedCallback(&Aci::parameterListUpdateFinished);
  aciSetSingleRequestReceivedCallback(&Aci::singleVariableReceived);

  startCommunicationThreads();

  if(!getDeviceInfo())
    return false;
  if(!getDeviceVariableList())
    return false;
  if(!getDeviceCommandList())
    return false;
  if(!getDeviceParameterList())
    return false;

  if(!detectFcuAndMavType(&fcu_type_, &mav_type_))
    return false;

  std::shared_ptr<SimpleTimeSynchronizer> time_synchronizer(new SimpleTimeSynchronizer());
  std::shared_ptr<LocalClockBase> time_measurement(new LocalClockBase());
  if (!setTimeSynchronizerAndLocalClock(time_synchronizer, time_measurement))
    return false;

  return true;
}

bool Aci::init(RawBufferPtr raw_buffer, std::shared_ptr<TimeSyncBase> time_synchronizer,
               std::shared_ptr<LocalClockBase> local_clock) {

  if (!init(raw_buffer)) {
    return false;
  }

  if (!setTimeSynchronizerAndLocalClock(time_synchronizer, local_clock)) {
    return false;
  }

  return true;
}

Aci::~Aci() {
  shutdown();
}

void Aci::shutdown(){
  stopCommunicationThreads();
  stopTimeSynchronizationThread();
}

void Aci::engineThread() {
  while (true) {
    // return immediately when a shutdown is requested, instead of reading from a possibly broken file descriptor
    if (!engine_thread_running_)
      return;

    UniqueLock lock(mutex_);
    aciEngine();
    lock.unlock();

    usleep(1000000 / kEngineRate);
  }
}

void Aci::rxThread() {
  uint8_t buffer[100];

  while (true) {
    // return immediately when a shutdown is requested, instead of reading from a possibly broken file descriptor
    if (!rx_thread_running_)
      return;

    // Blocks until data is available. Call is interruptible.
    int bytes_received = raw_buffer_->readBuffer(buffer, 100);

    if (bytes_received > 0) {
//    printf("\n %f ----- read %d bytes \n", get_time(), bytes_received);
      UniqueLock lock(mutex_);
      for (int i = 0; i < bytes_received; ++i) {
        aciReceiveHandler(buffer[i]);
      }
    }
  }
}

void Aci::timeSynchronizationThread() {
  using std::chrono::high_resolution_clock;
  using std::chrono::duration;

  typedef std::chrono::duration<double> DurationDouble;

  if (time_synchronizer_)
    time_synchronizer_->reset();
  else{
    ACI_WARN_STREAM("Time-synchronizer not set");
    return;
  }

  if (!local_clock_){
    ACI_WARN_STREAM("Time-measurement not set");
    return;
  }

  initialized_ = false;

  bool message_once = true;

  while (true) {
    if (!time_synchronization_thread_running_)
      return;

    bool request_successful = false;
    const double local_time_sent = local_clock_->now();
    const auto local_time_sent_chrono = std::chrono::high_resolution_clock::now();
    double device_time_received = 0;
    double device_time_sent = 0;

    if(fcu_type_ == FcuType::AutoPilot){
      Variable<VariableInt32> _device_time;
      // TODO(acmarkus): this is not standard on the sdk --> find other solution, or would asctec integrate
      // this? Hence the low ID, which is actually in asctec's address space.
      const int autopilot_ms_counter = 0x006;
      request_successful = requestSingleVariable(_device_time, autopilot_ms_counter,
                                                 std::chrono::milliseconds(100));
      device_time_received = static_cast<double>(_device_time.value()) * 1.0e-3;
      device_time_sent = device_time_received;
      if(!request_successful){
        ACI_ERROR_STREAM("Unable to request variable \"HL_up_time_ms\". This variable needs a modified "
            "firmware, e.g. https://github.com/ethz-asl/ethzasl_mav_firmware");
      }
    }
    else if (fcu_type_ == FcuType::Trinity) {
      Variable<VariableInt64> _device_time;

      request_successful = requestSingleVariable(_device_time, ACI_USER_VAR_USCOUNTER,
                                                 std::chrono::milliseconds(100));
      device_time_received = static_cast<double>(_device_time.value()) * 1.0e-6;
      device_time_sent = device_time_received;
    }

    // We use the user-defined clock as starting point, but time the duration until reception of the ping
    // with the high-resolution clock, where we have better control.
    const auto local_time_received_chrono = std::chrono::high_resolution_clock::now();
    const double roundtrip_time = std::chrono::duration_cast<std::chrono::duration<double>>(
        local_time_received_chrono - local_time_sent_chrono).count();

    const double local_time_received = local_time_sent + roundtrip_time;
    ACI_DEBUG_STREAM("Timesync round-trip:" << std::setprecision(18) << roundtrip_time);

    UniqueLock lock(time_synchronization_mutex_);
    time_synchronizer_->addRoundTripMeasurement(local_time_sent, device_time_received,
                                                device_time_sent, local_time_received);

    bool time_synchronizer_initialized = false;
    if (time_synchronizer_->getInitializationStatus() < 100) {
      ACI_INFO_STREAM("Initializing time synchronization: " <<
                      time_synchronizer_->getInitializationStatus() << "%");
    }
    else if (time_synchronizer_->getInitializationStatus() >= 100) {
      if (message_once) {
        ACI_INFO_STREAM("Initializing time synchronization: done ");
        message_once = false;
      }
      time_synchronizer_initialized = true;
    }

    // TODO(acmarkus): decide whether it's a good idea to to this after every measurement.
    // TODO(acmarkus): for now, we trust that the mapping function is smooth enough (handled in TimeSyncBase),
    //                 such that converted device times will not go backwards. If it happens, an error will be returned and we drop the measurement.
    if (time_synchronizer_initialized) {
      bool success = time_synchronizer_->update();
      ACI_ERROR_STREAM_COND(!success, "could not update time synchronization");
    }

    initialized_ = time_synchronizer_initialized;
    lock.unlock();

    std::this_thread::sleep_for(std::chrono::milliseconds(kTimesyncWaitTimeMs));
  }
}

Subscription Aci::registerVariableCallback(int packet_id, const VarPacketCallback& callback) {
  if (packet_id < variable_subscriptions_.size()) {
    variable_subscriptions_[packet_id].push_back(callback);
    VarPacketSubscription it = variable_subscriptions_[packet_id].end();
    --it;
    return Subscription(new SubscriptionObj(packet_id, it));
  }
  else {
    ACI_ERROR_STREAM(
        "" << __func__ << ": packet id > MAX_VAR_PACKETS: " << packet_id << " / " << variable_subscriptions_.size());
    return Subscription();
  }
}

void Aci::unregisterVariableCallback(int packet_id, const VarPacketSubscription& sub) {
  if (packet_id < variable_subscriptions_.size()) {
    variable_subscriptions_[packet_id].erase(sub);
  }
  else {
    ACI_ERROR_STREAM(
        "" << __func__ << ": packet id > MAX_VAR_PACKETS:" << packet_id << " / " << variable_subscriptions_.size());
    return;
  }
}

void Aci::setVariablePacketRate(int packet_id, double rate) {
  if (packet_id > MAX_VAR_PACKETS) {
    ACI_ERROR_STREAM(
        "" << __func__ << ": packet id > MAX_VAR_PACKETS:" << packet_id << " / " << MAX_VAR_PACKETS);
    return;
  }

  unsigned short period = 1000;

  // From the documentation, it seems like they mean the period of the calls:
  // 1000 / rate = "calls per second" (= the second parameter below).
  // I suspect that 1000 denotes the default controller cycles / s on the autopilot, while it's 500 on the trinity.
  if (rate > 0)
    if (fcu_type_ == FcuType::Trinity)
      period = static_cast<unsigned short>(kDefaultTrinityRate / rate);
    else if (fcu_type_ == FcuType::AutoPilot)
      period = static_cast<unsigned short>(kDefaultAutoPilotRate / rate);
    else {
      ACI_ERROR_STREAM("FCU type unknown, cannot set proper rate.");
      return;
    }
  else
    period = 1;
  aciSetVarPacketTransmissionRate(packet_id, period);

  aciVarPacketUpdateTransmissionRates();
}

void Aci::updateVariableConfiguration(int packet_id) {
  if (packet_id > MAX_VAR_PACKETS || packet_id < 0) {
    ACI_ERROR_STREAM("" << __func__ << ": packet id should be in [0 ... " << MAX_VAR_PACKETS
                     << "[ but is: " << packet_id);
    return;
  }
  aciSendVariablePacketConfiguration(packet_id);
}

void Aci::updateCommandConfiguration(int packet_id, bool with_acknowledgement) {
  if (packet_id > MAX_VAR_PACKETS || packet_id < 0) {
    ACI_ERROR_STREAM("" << __func__ << ": packet id should be in [0 ... " << MAX_VAR_PACKETS
                     << "[ but is: " << packet_id);
    return;
  };
  aciSendCommandPacketConfiguration(packet_id, with_acknowledgement ? 1 : 0);
}

int Aci::getNextFreeCommandPacketId() const {
  if (free_command_packet_ids_.empty()) {
    return -1;
  }
  else
    return free_command_packet_ids_.front();
}

void Aci::resetCommandPacket(int packet_id){
  if (packet_id > MAX_VAR_PACKETS || packet_id < 0) {
    ACI_ERROR_STREAM("" << __func__ << ": packet id should be in [0 ... " << MAX_VAR_PACKETS
                     << "[ but is: " << packet_id);
    return;
  };
  aciResetCmdPacketContent(packet_id);
  aciSendCommandPacketConfiguration(packet_id, 0);
  free_command_packet_ids_.push_back(packet_id);
  free_command_packet_ids_.sort();
}

void Aci::sendCommandPacketToDevice(int packet_id) {
  if (packet_id > MAX_VAR_PACKETS || packet_id < 0) {
    ACI_ERROR_STREAM("" << __func__ << ": packet id should be in [0 ... " << MAX_VAR_PACKETS
                     << "[ but is: " << packet_id);
    return;
  };
  aciUpdateCmdPacket(packet_id);
}

bool Aci::getDeviceInfo() {
  ACI_INFO_STREAM("Requesting version info from device ...");
  aciCheckVerConf();
  const double req_timeout = 5;

  int timeout = req_timeout * 1e6;
  while (!have_device_info_ && timeout) {
    const int wait_time = 1e5;  // 100ms
    usleep(wait_time);
    timeout -= wait_time;
  }

  if (timeout <= 0) {
    ACI_ERROR_STREAM("Unable to get version info within " << req_timeout << "s.");
    return false;
  }
  else
    ACI_INFO_STREAM("... version info received");

  bool major_versions_match = device_version_info_.verMajor == ACI_VER_MAJOR;
  bool minor_versions_match = device_version_info_.verMinor == ACI_VER_MINOR;
  bool number_of_var_packets_match = device_version_info_.maxVarPackets == MAX_VAR_PACKETS;

  if(!(major_versions_match && minor_versions_match && number_of_var_packets_match)){
    ACI_ERROR_STREAM("Version mismatch between device and remote. See info below");
    printVersionInfo();
    // TODO(acmarkus): fix this
//    std::exit(EXIT_FAILURE);
  }

  return true;
}

bool Aci::getDeviceVariableList() {
  ACI_INFO_STREAM("Requesting variable info from device ...");
  aciGetDeviceVariablesList();
  const double req_timeout = 7;

  int timeout = req_timeout * 1e6;
  while (!have_device_variables_ && timeout) {
    int wait_time = 1e5;  // 100ms
    usleep(wait_time);
    timeout -= wait_time;
  }

  if (timeout <= 0) {
    ACI_ERROR_STREAM("Unable to get variable info within " << req_timeout << "s.");
    return false;
  }
  else
    ACI_INFO_STREAM("... variable info received");

  return true;
}

bool Aci::getDeviceCommandList() {
  ACI_INFO_STREAM("Requesting command info from device ...");
  aciGetDeviceCommandsList();
  const double req_timeout = 5;

  int timeout = req_timeout * 1e6;
  while (!have_device_commands_ && timeout) {
    int wait_time = 1e5;  // 100ms
    usleep(wait_time);
    timeout -= wait_time;
  }

  if (timeout <= 0) {
    ACI_ERROR_STREAM("Unable to get command info within " << req_timeout << "s.");
    return false;
  }
  else
    ACI_INFO_STREAM("... command info received");

  return true;
}

bool Aci::getDeviceParameterList() {
  ACI_INFO_STREAM("Requesting parameter info from device ...");
  aciGetDeviceParametersList();
  const double req_timeout = 5;

  int timeout = req_timeout * 1e6;
  while (!have_device_parameters_ && timeout) {
    int wait_time = 1e5;  // 100ms
    usleep(wait_time);
    timeout -= wait_time;
  }

  if (timeout <= 0) {
    ACI_ERROR_STREAM("Unable to get parameter info within " << req_timeout << "s.");
    return false;
  }
  else
    ACI_INFO_STREAM("... parameter info received");

  return true;
}

void Aci::versionInfoCallback(struct ACI_INFO aci_info) {
  Aci& aci = Aci::instance();
  aci.have_device_info_ = true;

  aci.device_version_info_ = aci_info;

  ACI_INFO_STREAM("got version info\n");
}

void Aci::variableListUpdateFinished(void) {
  Aci& aci = Aci::instance();
  aci.have_device_variables_ = true;
}

void Aci::printVersionInfo() const {
  printf("******************** Versions *******************\n");
  printf("* Type\t\t\tDevice\t\tRemote\t*\n");
  printf("* Major version\t\t%d\t=\t\%d\t*\n", device_version_info_.verMajor, ACI_VER_MAJOR);
  printf("* Minor version\t\t%d\t=\t\%d\t*\n", device_version_info_.verMinor, ACI_VER_MINOR);
  printf("* MAX_DESC_LENGTH\t%d\t=\t\%d\t*\n", device_version_info_.maxDescLength, MAX_DESC_LENGTH);
  printf("* MAX_NAME_LENGTH\t%d\t=\t\%d\t*\n", device_version_info_.maxNameLength, MAX_NAME_LENGTH);
  printf("* MAX_UNIT_LENGTH\t%d\t=\t\%d\t*\n", device_version_info_.maxUnitLength, MAX_UNIT_LENGTH);
  printf("* MAX_VAR_PACKETS\t%d\t=\t\%d\t*\n", device_version_info_.maxVarPackets, MAX_VAR_PACKETS);
  printf("*************************************************\n");
}

void Aci::printVariableInfo() {
  const size_t len = aciGetVarTableLength();
  constexpr int kNameWidth = 32;
  constexpr int kDescWidth = 35;
  constexpr int kIdWidth   =  6;
  constexpr int kUnitWidth =  8;

  std::cout << std::setfill(' ');
  std::cout << "************************************ Variable info ************************************" << std::endl;
  std::cout << "* " << std::left << std::setw(kNameWidth) << "Name"
                    << std::setw(kDescWidth) << "Description"
                    << std::setw(kIdWidth)   << "ID"
                    << std::setw(kUnitWidth) << "  Unit"
                    << "   *" << std::endl;

  for (size_t i = 0; i < len; ++i) {
    ACI_MEM_TABLE_ENTRY* it = aciGetVariableItemByIndex(i);
    std::cout << "* " << std::left << std::setw(kNameWidth) << it->name
                      << std::setw(kDescWidth) << it->description
                      << "0x" << std::hex << std::setw(kIdWidth) << it->id
                      << std::setw(kUnitWidth) << it->unit
                      << " *" << std::endl;
  }
  std::cout << "***************************************************************************************" << std::endl;
}

void printMemTableEntryDefine(ACI_MEM_TABLE_ENTRY* entry) {
  std::string name(entry->name);
  transform(name.begin(), name.end(), name.begin(), toupper);
  if (entry->id < 0x1000)
    name = "ACI_" + name;
  else
    name = "CUSTOM_" + name;

  std::cout << "#define " << name << " 0x" << std::hex << (int) entry->id << " " << std::endl;
}

void Aci::printVariableDefines() {
  const size_t len = aciGetVarTableLength();
  for (size_t i = 0; i < len; ++i) {
    ACI_MEM_TABLE_ENTRY* it = aciGetVariableItemByIndex(i);
    printMemTableEntryDefine(it);
  }
}

void Aci::printCommandDefines() {
  // no call for getting the number of commands :(
  ACI_MEM_TABLE_ENTRY* it;
  int i=0;
  while (true) {
    ACI_MEM_TABLE_ENTRY* it = aciGetCommandItemByIndex(i);
    if(it == NULL)
      return;
    printMemTableEntryDefine(it);
    ++i;
  }
}

void Aci::commandListUpdateFinished(void) {
  Aci& aci = Aci::instance();
  aci.have_device_commands_ = true;
}

void Aci::parameterListUpdateFinished(void) {
  Aci& aci = Aci::instance();
  aci.have_device_parameters_ = true;
}

void Aci::variablePacketUpdated(unsigned char id) {
  Aci& aci = Aci::instance();

  if (!aci.initialized_) {
    return;
  }

//  printf("got packet %d\n", (int) id);
  aciSynchronizeVars();

  size_t size = aci.variable_subscriptions_.size();

  if (id < size) {
    VarPacketSubscriptionList& list = aci.variable_subscriptions_[id];
    for (VarPacketCallback& cb : list) {
      cb();
    }
//    ACI_INFO_STREAM("called "<< list.size() << "subscribers");
  }
  else {
    ACI_ERROR_STREAM("Packet index out of range: " << static_cast<int>(id) << "/" << size);
  }
}

void Aci::singleVariableReceived(unsigned short id, void *data, unsigned char varType) {
  // Gets called from the rx thread, which also takes care of locking/unlocking the mutex.
//  ACI_INFO_STREAM("single variable received: id = 0x" << std::hex << (int) id << " type: 0x"<< (int) varType);
  Aci& aci = Aci::instance();
  size_t size = 0x3f & (varType >> 2);
  memcpy(aci.single_request_buffer_, data, size);
  aci.single_request_returned_id_ = id;
  aci.single_request_condition_.notify_all();
}

bool Aci::detectFcuAndMavType(FcuType* fcu_type, MavType* mav_type) {
  if (fcu_type == nullptr || mav_type == nullptr)
    return false;

  *fcu_type = FcuType::Unknown;
  *mav_type = MavType::Unknown;

  char variable_name_autopilot[] = "HL_cpu_load";
  char variable_name_trinity[] = "us counter";

  if (aciGetVariableItemByName(variable_name_autopilot)) {
    *fcu_type = FcuType::AutoPilot;
    ACI_INFO_STREAM("Detected FCU AutoPilot");
  }
  else if (aciGetVariableItemByName(variable_name_trinity)) {
    *fcu_type = FcuType::Trinity;
    ACI_INFO_STREAM("Detected FCU Trinity");
  }
  else {
    ACI_ERROR_STREAM("Could not detect FCU");
    return false;
  }

  aci::Variable<aci::VariableUint8> _mav_type;
  bool request_successful = requestSingleVariable(_mav_type, ACI_USER_VAR_VEHICLE_TYPE,
                                                  std::chrono::milliseconds(100));
  if (!request_successful) {
    ACI_ERROR_STREAM("Request to get MAV type failed");
    return false;
  }

  std::cout<< "DBG: " << static_cast<int>(_mav_type.value())<< std::endl;

  switch (_mav_type.value()) {
    case ACI_VEHICLE_TYPE_FALCON:
      *mav_type = MavType::Falcon;
      break;
    case ACI_VEHICLE_TYPE_FIREFLY:
      *mav_type = MavType::Firefly;
      break;
    case ACI_VEHICLE_TYPE_NEO_6_9:
      *mav_type = MavType::Neo_6_9;
      break;
    case ACI_VEHICLE_TYPE_NEO_6_11:
      *mav_type = MavType::Neo_6_11;
      break;
    case ACI_VEHICLE_TYPE_HUMMINGBIRD:
      *mav_type = MavType::Hummingbird;
      break;
    default:
      ACI_ERROR_STREAM("Unable to detect MAV type.");
      return false;
  }

  ACI_INFO_STREAM("Detected MAV " << mavTypeToString(*mav_type));
  return true;
}

bool Aci::setTimeSynchronizer(std::shared_ptr<TimeSyncBase> time_synchronizer) {
  if (!time_synchronizer) {
    ACI_ERROR_STREAM("time_synchronizer is NULL");
    return false;
  }

  stopTimeSynchronizationThread();
  time_synchronizer_ = time_synchronizer;
  time_synchronizer_->reset();

  if (time_synchronizer_ && local_clock_)
    startTimeSynchronizationThread();

  return true;
}

bool Aci::setLocalClock(std::shared_ptr<LocalClockBase> local_clock) {
  if (!local_clock) {
    ACI_ERROR_STREAM("time_measurement is NULL");
    return false;
  }

  stopTimeSynchronizationThread();
  local_clock_ = local_clock;

  if (time_synchronizer_) {
    time_synchronizer_->reset();
  }

  if (time_synchronizer_ && local_clock_) {
    startTimeSynchronizationThread();
  }

  return true;
}

bool Aci::setTimeSynchronizerAndLocalClock(std::shared_ptr<TimeSyncBase> time_synchronizer,
                                           std::shared_ptr<LocalClockBase> local_clock) {
  if (!time_synchronizer) {
    ACI_ERROR_STREAM("time_synchronizer is NULL");
    return false;
  }

  if (!local_clock) {
    ACI_ERROR_STREAM("time_measurement is NULL");
    return false;
  }

  stopTimeSynchronizationThread();
  time_synchronizer_ = time_synchronizer;
  time_synchronizer_->reset();
  local_clock_ = local_clock;
  startTimeSynchronizationThread();

  return true;
}

bool Aci::deviceTimeToLocalTime(double device_time, double* local_time){
  if (!time_synchronizer_) {
      ACI_ERROR_STREAM("time_synchronizer is NULL");
      return false;
    }

  UniqueLock lock(time_synchronization_mutex_);
  return time_synchronizer_->deviceTimeToLocalTime(device_time, local_time);
}

bool Aci::localTimeToDeviceTime(double local_time, double* device_time){
  if (!time_synchronizer_) {
      ACI_ERROR_STREAM("time_synchronizer is NULL");
      return false;
    }

  UniqueLock lock(time_synchronization_mutex_);
  return time_synchronizer_->localTimeToDeviceTime(local_time, device_time);
}

std::string mavTypeToString(const MavType& mav_type) {
  switch (mav_type) {
    case MavType::Falcon:
      return std::string("Falcon");
    case MavType::Firefly:
      return std::string("Firefly");
    case MavType::Neo_6_9:
      return std::string("Neo 6-9");
    case MavType::Neo_6_11:
      return std::string("Neo 6-11");
    case MavType::Hummingbird:
      return std::string("Hummingbird");
    default:
      return std::string("unknown");
  }
}

std::string flightModeToStringTrinity(uint32_t flight_mode) {
  std::stringstream ss;

  if(flight_mode & ACI_FLIGHTMODE_ACC)
    ss << "absolute angle control, ";
  if(flight_mode & ACI_FLIGHTMODE_POS)
    ss << "xy position control, ";
  if(flight_mode & ACI_FLIGHTMODE_FLYING)
    ss << "motors running, ";
  if(flight_mode & ACI_FLIGHTMODE_EMERGENCY)
    ss << "emergency, ";
  if(flight_mode & ACI_FLIGHTMODE_TRAJECTORY)
    ss << "automatic navigation, ";
  if(flight_mode & ACI_FLIGHTMODE_HEIGHT)
    ss << "altitude control, ";
  if(flight_mode & ACI_FLIGHTMODE_MOTOR_CURRENT_CALIB)
    ss << "motor current_calibration running, ";
  if(flight_mode & ACI_FLIGHTMODE_AUTO_COMPASS_CALIB)
    ss << "auto compass calibration_running, ";
  if(flight_mode & ACI_FLIGHTMODE_HOVER_CALIB)
    ss << "hover calibration running, ";

  std::string ret_string = ss.str();
  ret_string.erase(ret_string.find_last_of(","), 1);

  return ret_string;
}

} // end namespace aci
