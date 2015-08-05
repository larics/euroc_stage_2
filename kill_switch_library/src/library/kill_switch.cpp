#include "kill_switch_library/kill_switch.h"

namespace kill_switch_library {

KillSwitch::KillSwitch(double check_frequency_hz)
    : check_char_(0),
      kill_status_(false),
      check_frequency_hz_(check_frequency_hz),
      checkingThread_(nullptr),
      stop_(false),
      started_(false),
      connected_(false)
{
  // The timeout is half the time required by the checking frequency.
  // For example for a checkfreq of 100Hz we have a timeout of 5ms.
  timeout_ms_ = (1.0 * 1000.0) / (check_frequency_hz * 2);
}

KillSwitch::~KillSwitch()
{
  stop();
}

bool KillSwitch::connect(const std::string& port, int baudrate)
{
  // Connecting to the switch
  bool success = uart_.connect(port, baudrate);
  // Setting flag indicating switch is connected to hardware
  if (success) {
    connected_ = true;
    ROS_INFO("Kill switch connected.");
    return true;
  } else {
    ROS_WARN("Kill switch could not be connected");
    connected_ = false;
    return false;
  }
}

bool KillSwitch::start()
{
  // If not already started 
  if (!started_) {
    // If connected to hardware
    if (connected_) {
      if (kill_status_ == false) {
        // Create thread to do switch checking
        stop_ = false;
        checkingThread_ = new std::thread(&KillSwitch::checkLoop, this);
        started_ = true;
        ROS_INFO("Kill switch started.");
        return true;
      } else {
        ROS_WARN("Killswitch already triggered and thus not started. Reset.");
        return false;
      }
    } else {
      ROS_WARN("Killswitch not started. Connect to serial port.");
      return false;
    }
  } else {
    return true;
  }
}

void KillSwitch::stop()
{
  // If already started, stop the checking thread
  if (started_ == true) {
    stop_ = true;
    if (checkingThread_->joinable())
      checkingThread_->join();
    checkingThread_ = nullptr;
    started_ = false;
  }
}

bool KillSwitch::reset()
{
  // Resetting the kill status
  if (kill_status_ == true) {
    // If switch not depressed
    if (check() == false) {
      kill_status_ = false;
      ROS_INFO("Kill switch reset.");
      return true;
    } else {
      ROS_WARN("Cannot reset kill switch. Still triggered.");
      return false;
    }
  } else {
    ROS_INFO("Switch not triggered.");
    return true;
  }
}

void KillSwitch::checkLoop()
{
  // Looping forever and printing
  ros::Rate loop_rate(check_frequency_hz_);
  while (ros::ok() && !stop_) {
    // Checking the switch
    if (check()) {
      kill_status_ = true;
      started_ = false;
      break;
    }
    // Sleeping until next execution
    loop_rate.sleep();
  }
}

bool KillSwitch::check()
{
  // Writing a character on to the switch
  check_char_++;
  int num_bytes_writen = uart_.writeBuffer((uint8_t*) &check_char_, 1);
  // Checking bytes written
  if (num_bytes_writen != 1)
    return true;  //TODO(millanea):  If were unable to write to the uart what is the appropriate action?
                  //                 Here I have assumed the appropriate action is to trigger the kill
                  //                 switch. This is the safest option.

  // Reading the character back
  char check_char_read;
  int num_bytes_read = uart_.readBuffer((uint8_t*) &check_char_read, 1, timeout_ms_);
  // Performing check
  bool char_received_check = num_bytes_read == 1;
  bool correct_char_received_check = check_char_read == check_char_;
  if (char_received_check && correct_char_received_check)
    return false;
  else
    return true;
}

}

