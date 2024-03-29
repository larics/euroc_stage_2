/*
 * Copyright (c) 2011-2014, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>
 *
 * Serial port setup parts shamelessly stolen from
 * https://github.com/mavlink/c_uart_interface_example/blob/master/mavlink_serial.cpp
 * Copyright (c) 2014, Lorenz Meier, CVG, ETH Zurich, Switzerland
 * You can contact the author at <lm at inf dot ethz dot ch>
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

#ifndef UART_H
#define UART_H

#include <iostream>
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>  /* UNIX standard function definitions */

#include <ros/console.h> /* File control definitions */

#include <mutex>
#include <thread>
#include <condition_variable>

class WriteQueue {
 public:
  enum {
    WAIT_TIMEOUT =
        1000,  ///< Timeout for waiting for new data in the buffer, in [ms].
    WRITE_BUFFER_SIZE = 1024
  };
  WriteQueue();
  ~WriteQueue();
  void reset(int fd);
  void stop();
  int writeBuffer(uint8_t* data, int size);

 private:
  typedef std::unique_lock<std::mutex> UniqueLock;
  int fd_;
  int pos_;
  uint8_t buffer_[WRITE_BUFFER_SIZE];
  std::condition_variable cond_;
  std::mutex mutex_;
  std::thread write_thread_;
  bool shutdown_;
  void writeThread();
};

/**
 * Can connect to either one or two (e.g. for separate rx/tx wireless links)
 * serial ports.
 * The default baudrate can be changed up to 921600 baud. Messages can be sent
 * with
 * Uart::sendPacket or Uart::sendPacketAck . Messages are received by setting a
 * callback function
 * with Uart::registerCallback for the message id to receive.
 */
class Uart {
 public:
  Uart();
  ~Uart();

  /// connects to the specified serial port(s) with the given baudrate. The HLP
  /// sets it's baudrate automatically.
  /**
   * The port names can be equal, then it connects only to one serial port (most
   * common operation).
   * It can also connect to different ports for rx and tx. This is useful when
   * e.g. two wireless modules (one for rx, one for tx) such as XBee are used to
   * connect
   * to the helicopter and bandwidth of one link is to low. rx/tx is seen from
   * the host computer running the communication node. Any baudrate can be set.
   * If it doesn't
   * match a standard baudrate, the closest standard baudrate is chosen. If the
   * HLP doesn't "hear" anything from the node for ~10s, it will go into
   * autobaud mode. During connection
   * setup, the node sends 'a' to the HLP, which will then configure its
   * baudrate automatically.
   * @param port_rx port to use for rx
   * @param port_tx port to use for tx
   * @param baudrate baudrate to connect with. equal for both ports. The
   * baudrate finally chosen is written to baudrate.
   * @return connection successful
   */
  bool connect(const std::string& port_rx, const std::string& port_tx,
               int baudrate);
  bool connect(const std::string& port, int baudrate);

  /// Closes the serial port(s).
  void closePort();

  // Writing to the transmission queue
  int writeBuffer(uint8_t* data, int size);

  // Reading port
  int readBuffer(uint8_t* data, int size, int timeout_ms = 1000);

  // Added for unit testing
  int getPtyfd() { return fd_rx_; }

 private:
  int openPort(const std::string& port);
  bool setupPort(int fd, int* baudrate, int data_bits = 8, int stop_bits = 1,
                 bool parity = false, bool hardware_control = false);
  bool connect(const std::string& port, int baudrate, int* serial_port);

  // The file descriptors to transmission and reception
  int fd_rx_;
  int fd_tx_;

  bool ok_;

  std::string port_rx_name_;
  std::string port_tx_name_;

  // Queue managing writing on the port
  WriteQueue tx_queue_;
};

#endif  // SERIAL_PORT_H
