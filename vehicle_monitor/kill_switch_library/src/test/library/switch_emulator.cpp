#include "switch_emulator.h"

SwitchEmulator::SwitchEmulator(double loop_frequency_hz)
    : kill_state_(false),
      connected_(false),
      started_(false),
      stop_(true),
      loop_frequency_hz_(loop_frequency_hz)
{

}

bool SwitchEmulator::connect(const std::string& port, int baudrate)
{
  // Connecting to the switch
  bool success = uart_.connect(port, baudrate);
  // Setting flag indicating switch is connected to hardware
  if (success) {
    connected_ = true;
    ROS_INFO("Kill switch connected.");
    return true;
  } else {
    ROS_ERROR("Kill switch could not be connected");
    connected_ = false;
    return false;
  }
}

bool SwitchEmulator::start()
{
  // If not already started 
  if (!started_) {
    if (connected_) {
      // Create thread to do switch checking
      stop_ = false;
      switch_thread_ = new std::thread(&SwitchEmulator::switchLoop, this);
      started_ = true;
      ROS_INFO("SwitchEmulator switch started.");
      return true;
    } else {
      ROS_ERROR("SwitchEmulator not started. Connect to serial port.");
      return false;
    }
  } else {
    return true;
  }
}

void SwitchEmulator::stop()
{
  // If already started, stop the checking thread
  if (started_ == true) {
    stop_ = true;
    if (switch_thread_->joinable())
      switch_thread_->join();
    switch_thread_ = nullptr;
    started_ = false;
  }
}

void SwitchEmulator::trigger()
{
  kill_state_ = true;
}

void SwitchEmulator::untrigger()
{
  kill_state_ = false;
}

void SwitchEmulator::switchLoop()
{
  while (ros::ok() && !stop_) {
    // Reading and character
    char switch_char;
    int timeout = 100;  // [ms]
    int num_bytes_read = uart_.readBuffer((uint8_t*) &switch_char, 1, timeout);
    if (num_bytes_read > 0) {
      // If not triggered sending the same character back
      if (kill_state_ == false) {
        int num_bytes_writen;
        num_bytes_writen = uart_.writeBuffer((uint8_t*) &switch_char, 1);
      }
    }
  }
}
