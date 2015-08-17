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

#include "kill_switch_library/uart.h"

#ifdef __APPLE__

#ifndef B460800
#define B460800 460800
#endif

#ifndef B921600
#define B921600 921600
#endif

#endif

Uart::Uart()
    : fd_rx_(-1),
      fd_tx_(-1),
      ok_(true)
{

}

Uart::~Uart()
{
  ok_ = false;
  this->closePort();
}

void Uart::closePort()
{
  // Closing file descriptors pointing to ports
  ::close(fd_rx_);
  if (port_rx_name_ != port_tx_name_)
    ::close(fd_tx_);
}

// Public connection method for single line (normal) operation
bool Uart::connect(const std::string & port, int baudrate)
{
  // Internally use multiline connection method
  return connect(port, port, baudrate);
}

// Public connection method for seperate TX and RX communication lines
bool Uart::connect(const std::string & port_rx, const std::string & port_tx, int baudrate)
{
  bool success = false;
  port_tx_name_ = port_tx;
  port_rx_name_ = port_rx;

  if (port_rx == port_tx) {
    success = connect(port_rx, baudrate, &fd_rx_);
    fd_tx_ = fd_rx_;
  } else {
    if (connect(port_rx, baudrate, &fd_rx_)) {
      success = connect(port_rx, baudrate, &fd_tx_);
    }
  }

  if (!success)
    return false;

  // Starting queue for writing on the file descriptor
  tx_queue_.reset(fd_tx_);

  // write packet that configures autobaud. LSB needs to be 1, LSB+1 needs to be 0
//  uint8_t buf = 'a';
//  tx_queue_.writeBuffer(&buf, 1);

  return true;
}

// Opens and returns the file describing the serial port
int Uart::openPort(const std::string& port)
{
  int fd;

  // Open serial port
  // O_RDWR - Read and write
  // O_NOCTTY - Ignore special chars like CTRL-C
  fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1) {
    return (-1);
  } else {
    fcntl(fd, F_SETFL, 0 | O_NONBLOCK);
  }

  return (fd);
}

// Private connection function called by both public connection functions
bool Uart::connect(const std::string & port, int baudrate, int* serial_port)
{
  *serial_port = openPort(port);

  if (*serial_port == -1) {
    ROS_ERROR_STREAM("Could not open port "<< port);  //TODO(millanea): replace!
    return false;
  }

  if (!setupPort(*serial_port, &baudrate))
    return false;

  ROS_INFO_STREAM("Opened serial port " << port << " with baudrate " << baudrate);  //TODO(millanea): replace!
  return true;
}

bool Uart::setupPort(int port_fd, int* baudrate, int data_bits, int stop_bits, bool parity,
                     bool hardware_control)
{

  // Finding valid baudrate closest to requested baudrate
  int baudrates[] = { 9600, 14400, 19200, 38400, 57600, 115200, 230400, 460800, 921600 };
  int best_baudrate = 57600;
  int min_diff = 1e6;
  for (int i = 0; i < sizeof(baudrates) / sizeof(int); i++) {
    int diff = abs(baudrates[i] - *baudrate);
    if (diff < min_diff) {
      min_diff = diff;
      best_baudrate = baudrates[i];
    }
  }

  if (best_baudrate != *baudrate)
    ROS_ERROR_STREAM(
        "Unsupported baudrate, choosing closest supported baudrate: " << best_baudrate);

  *baudrate = best_baudrate;

  struct termios config;
  if (!isatty(port_fd)) {
    ROS_ERROR_STREAM("File descriptor "<< port_fd <<" is NOT a serial port\n");
    return false;
  }
  if (tcgetattr(port_fd, &config) < 0) {
    ROS_ERROR_STREAM(
        "Could not read configuration of fd " << port_fd << ". Reason: " << strerror(errno));
    return false;
  }

  // Input flags - Turn off input processing
  // convert break to null byte, no CR to NL translation,
  // no NL to CR translation, don't mark parity errors or breaks
  // no input parity check, don't strip high bit off,
  // no XON/XOFF software flow control
  config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

  // Output flags - Turn off output processing
  // no CR to NL translation, no NL to CR-NL translation,
  // no NL to CR translation, no column 0 CR suppression,
  // no Ctrl-D suppression, no fill characters, no case mapping,
  // no local output processing
  config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

#ifdef OLCUC
  config.c_oflag &= ~OLCUC;
#endif

#ifdef ONOEOT
  config.c_oflag &= ~ONOEOT;
#endif

  // No line processing:
  // echo off, echo newline off, canonical mode off,
  // extended input processing off, signal chars off
  config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

  // Turn off character processing
  // clear current char size mask, no parity checking,
  // no output processing, force 8 bit input
  config.c_cflag &= ~(CSIZE | PARENB);
  config.c_cflag |= CS8;

  // Ignore CD line. Needed for OSX users (http://stackoverflow.com/a/24374870) but be done in linux as well.
  config.c_cflag |= CLOCAL;

  // One input byte is enough to return from read() TODO: check / reconsider
  // Inter-character timer off
  config.c_cc[VMIN] = 1;
  config.c_cc[VTIME] = 10;  // was 0

  // Get the current options for the port
  //tcgetattr(fd, &options);

  bool success = false;

  switch (best_baudrate) {
    case 1200:
      success = (cfsetispeed(&config, B1200) == 0 && cfsetospeed(&config, B1200) == 0);
      break;
    case 1800:
      success = (cfsetispeed(&config, B1800) == 0 && cfsetospeed(&config, B1800) == 0);
      break;
    case 9600:
      success = (cfsetispeed(&config, B9600) == 0 && cfsetospeed(&config, B9600) == 0);
      break;
    case 19200:
      success = (cfsetispeed(&config, B19200) == 0 && cfsetospeed(&config, B19200) == 0);
      break;
    case 38400:
      success = (cfsetispeed(&config, B38400) == 0 && cfsetospeed(&config, B38400) == 0);
      break;
    case 57600:
      success = (cfsetispeed(&config, B57600) == 0 && cfsetospeed(&config, B57600) == 0);
      break;
    case 115200:
      success = (cfsetispeed(&config, B115200) == 0 && cfsetospeed(&config, B115200) == 0);
      break;

      // These two non-standard (by the 70'ties ) rates are fully supported on
      // current Debian and Mac OS versions (tested since 2010).
    case 460800:
      success = (cfsetispeed(&config, B460800) == 0 && cfsetospeed(&config, B460800) == 0);
      break;
    case 921600:
      success = (cfsetispeed(&config, B921600) == 0 && cfsetospeed(&config, B921600) == 0);
      break;
    default:
      ROS_ERROR_STREAM("" << best_baudrate << " does not match any baudrate available.");
      return false;

      break;
  }

  if (!success) {
    ROS_ERROR_STREAM(
        "Baudrate " << best_baudrate << " could not be set. Reason: " << strerror(errno));
    return false;
  }

  // Apply the configuration.
  if (tcsetattr(port_fd, TCSAFLUSH, &config) < 0) {
    ROS_ERROR_STREAM(
        "Could not set configuration of fd " << port_fd << "Reason: " << strerror(errno));
    return false;
  }

  return true;
}

int Uart::writeBuffer(uint8_t* data, int size)
{
  return tx_queue_.writeBuffer(data, size);
}

int Uart::readBuffer(uint8_t* data, int size, int timeout_ms)
{
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(fd_rx_, &read_fds);

  // Waits for file to become readable or timeout
  struct timeval timeout;
  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = (timeout_ms - (timeout_ms / 1000) * 1000) * 1000;
  //int timeout_ms = timeout.tv_sec * 1000 + timeout.tv_usec / 1000;
  int res = select(fd_rx_ + 1, &read_fds, NULL, NULL, &timeout);

  if (!ok_)
    return 0;

  if (res == 1) {
    const int bytes_received = read(fd_rx_, data, size);
    return bytes_received;
  } else if (res == 0) {
    // ROS_INFO_STREAM("No data read within the last " << timeout_ms / 1000.0f << "s");
    return 0;
  } else {
    ROS_ERROR_STREAM("read error:" << res << " " << errno << " " << strerror(errno));
    return -1;
  }
}

WriteQueue::WriteQueue()
    : fd_(-1),
      pos_(0),
      shutdown_(true)
{

}

WriteQueue::~WriteQueue()
{
  stop();
}

void WriteQueue::reset(int fd)
{
  // Stops the current write thread
  stop();
  // Changes file descriptor we're writing to
  fd_ = fd;
  // Allows thread to restart
  shutdown_ = false;
  // Reset buffer position
  pos_ = 0;
  // Starts the writing thread
  write_thread_ = std::thread(&WriteQueue::writeThread, this);
}

void WriteQueue::stop()
{
  // Signals the write thread to stop and then waits for it to finish
  shutdown_ = true;
  if (write_thread_.joinable())
    write_thread_.join();
}

void WriteQueue::writeThread()
{
  // Loop until another thread shut down
  while (!shutdown_) {
    // Locking access to the queue
    UniqueLock lock(mutex_);
    std::cv_status status(std::cv_status::timeout);

    // While no data in buffer
    while (pos_ == 0) {
      // Wait until signaled that new data in buffer. Periodically check for shutdown
      status = cond_.wait_for(lock, std::chrono::milliseconds(WAIT_TIMEOUT));
      if (shutdown_)
        return;
    }

    // Write all new data to file descriptor
    int res = ::write(fd_, buffer_, pos_);

    if (res == -1)
      ROS_ERROR_STREAM("could not write " << pos_ << " bytes, reason: " << strerror(errno));

    // Reset buffer to empty state
    pos_ = 0;
  }
}

int WriteQueue::writeBuffer(uint8_t* data, int size)
{
  // Locking mutex controlling access to the queue (buffer_)
  UniqueLock lock(mutex_);

  // Usually we get whole data packets, so it wouldn't make sense to truncate the data here
  if (size > WRITE_BUFFER_SIZE - pos_) {
    ROS_ERROR_STREAM(
        "Send buffer full: only "<< WRITE_BUFFER_SIZE - pos_ << " bytes of "<< WRITE_BUFFER_SIZE << " bytes available, while trying to send " << size << "bytes.");
    return -1;
  }

  // Copying data into the buffer
  memcpy(&buffer_[pos_], data, size);
  pos_ += size;

  // Unlocking queue for write thread
  lock.unlock();
  // Notifying that the buffer contains data
  cond_.notify_one();

  // Returning num bytes written
  return size;
}
