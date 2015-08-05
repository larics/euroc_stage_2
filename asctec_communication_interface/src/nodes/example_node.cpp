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

#include <aci/aci.h>
#include <aci/raw_buffer.h>
#include <aci/uart.h>
#include <aci/variable_defines_autopilot.h>
#include <aci/variable_defines_trinity.h>

#include <aci/variable.h>

class DummyPort : public aci::RawBuffer {
 public:
  int writeBuffer(uint8_t* data, int size) {
    return 1;
  }
  int readBuffer(uint8_t* data, int size) {
    return 1;
  }
};

class ImuPublisher {
 public:
  ImuPublisher() {
    aci::Aci& aci = aci::instance();
    angle_roll_ = aci.registerVariable<aci::VariableInt32>(aci::VAR_PACKET_FAST, ACI_ANGLE_ROLL);
    angle_pitch_ = aci.registerVariable<aci::VariableInt32>(aci::VAR_PACKET_FAST, ACI_ANGLE_PITCH);
    angle_yaw_ = aci.registerVariable<aci::VariableInt32>(aci::VAR_PACKET_FAST, ACI_ANGLE_YAW);

    aci.updateVariableConfiguration(aci::VAR_PACKET_FAST);
    fast_packet_sub_ = aci.registerVariableCallback(aci::VAR_PACKET_FAST, &ImuPublisher::fastPacketCallback,
                                                    this);
  }
 private:
  void fastPacketCallback() {
    std::cout << "Attitude (NED): roll = " << angle_roll_.value() << " pitch = " << angle_pitch_.value()
              << " yaw = " << angle_yaw_.value() << std::endl;
  }

  aci::Variable<aci::VariableInt32> angle_roll_;
  aci::Variable<aci::VariableInt32> angle_pitch_;
  aci::Variable<aci::VariableInt32> angle_yaw_;

  aci::Subscription fast_packet_sub_;
};

int main(int argc, char ** argv) {
  if(!(argc == 1 || argc == 3)){
    std::cout << "usage: " << argv[0] << " port baudrate" << std::endl;
    std::cout << "    starts with the specified port with baudrate" << std::endl;
    std::cout << argv[0] << std::endl;
    std::cout << "    starts with a dummy serial port and won't do anything" << std::endl;
    return EXIT_FAILURE;
  }

  aci::RawBufferPtr serial_port;

  if (argc == 3) {
    std::string port_name(argv[1]);
    int baudrate = std::stoi(argv[2]);
    aci::UartPtr uart(new aci::Uart);
    bool port_ok = uart->connect(port_name, baudrate);

    if (!port_ok) {
      std::cout << "unable to set up serial port " << port_name << " with " << baudrate << "baud"
                << std::endl;
      return EXIT_FAILURE;
    }
    serial_port = uart;
  }
  else if(argc == 1){
    std::shared_ptr<DummyPort> dummy_port(new DummyPort);
    serial_port = dummy_port;
  }

  aci::Aci& aci = aci::instance();
  if(!aci.init(serial_port)){
    return EXIT_FAILURE;
  }

  aci.printVersionInfo();

  aci.setVariablePacketRate(aci::VAR_PACKET_FAST, 100.0);

  aci.printVariableInfo();

  ImuPublisher imu_pub;

  int packet_id = aci.getNextFreeCommandPacketId();
  aci::Variable<aci::VariableInt16> command_pitch = aci.registerCommand<aci::VariableInt16>(packet_id,
                                                                                            0x050A);
  aci.updateCommandConfiguration(packet_id, false);

  aci::Variable<aci::VariableInt32> test_single_variable;
  aci.requestSingleVariable(test_single_variable, ACI_ANGVEL_PITCH, std::chrono::milliseconds(200));
  std::cout << "single variable: " << test_single_variable.value() << std::endl;

  const int pitch_amplitude = 200;
  int pitch = -pitch_amplitude;
  while (true) {
    usleep(100000);

    command_pitch = pitch;
    aci.sendCommandPacketToDevice(packet_id);

    ++pitch;
    if (pitch > pitch_amplitude)
      pitch = -pitch_amplitude;
  }

  return 0;
}
