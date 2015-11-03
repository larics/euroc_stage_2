#include "kill_switch_library/kill_switch.h"

namespace kill_switch_library {

KillSwitch::KillSwitch(double check_frequency_hz, double wait_time_s)
    : check_char_(0),
      kill_status_(false),
      check_frequency_hz_(check_frequency_hz),
      checkingThread_(nullptr),
      stop_(false),
      started_(false),
      connected_(false),
      switch_state_(SAFE),
      safe_counter_(0),
      wait_time_s_(wait_time_s) {
  // The timeout is half the time required by the checking frequency.
  // For example for a checkfreq of 100Hz we have a timeout of 5ms.
  timeout_ms_ = (1.0 * 1000.0) / (check_frequency_hz * 2);
}

KillSwitch::~KillSwitch() { stop(); }

bool KillSwitch::connect(const std::string& port, int baudrate) {
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

bool KillSwitch::start() {
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

void KillSwitch::stop() {
  // If already started, stop the checking thread
  if (started_ == true) {
    stop_ = true;
    if (checkingThread_->joinable()) checkingThread_->join();
    checkingThread_ = nullptr;
    started_ = false;
  }
}

bool KillSwitch::reset() {
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

void KillSwitch::checkLoop() {
  // Looping forever and printing
  ros::Rate loop_rate(check_frequency_hz_);
  while (ros::ok() && !stop_) {
    // Taking action depending on switch state
    switch (switch_state_) {
      // Kill state
      case KILLED:
        // If switch saying safe - start transition
        if (check() == false) {
          switch_state_ = TRANS_TO_SAFE;
          //          ROS_INFO_STREAM("Entering transition to safe state. Wait:
          //          " << wait_time_s_ << " seconds.");
        }
        break;
      // Transitioning to safe state
      case TRANS_TO_SAFE:
        // If switch saying safe - keep counting
        if (check() == false) {
          safe_counter_++;
          // If count reached - transition to safe state
          if (safe_counter_ >=
              static_cast<int>(wait_time_s_ * check_frequency_hz_)) {
            kill_status_ = false;
            switch_state_ = SAFE;
          }
        }
        // If switch saying kill - transition to kill state + reset counter
        else {
          safe_counter_ = 0;
          switch_state_ = KILLED;
        }
        break;
      // Safe state
      case SAFE:
        // If switch saying kill - immediately transition to kill state
        if (check() == true) {
          kill_status_ = true;
          safe_counter_ = 0;
          switch_state_ = KILLED;
        }
        break;
    }
    // Sleeping until next execution
    loop_rate.sleep();
  }
}

bool KillSwitch::check() {
  // Writing a character on to the switch
  check_char_++;
  int num_bytes_writen = uart_.writeBuffer((uint8_t*)&check_char_, 1);
  // Checking bytes written
  if (num_bytes_writen != 1)
    return true;  // TODO(millanea):  If were unable to write to the uart what
                  // is the appropriate action?
  //                 Here I have assumed the appropriate action is to trigger
  //                 the kill
  //                 switch. This is the safest option.

  // Reading the character back
  char check_char_read;
  int num_bytes_read =
      uart_.readBuffer((uint8_t*)&check_char_read, 1, timeout_ms_);
  // Performing check
  bool char_received_check = num_bytes_read == 1;
  bool correct_char_received_check = check_char_read == check_char_;
  if (char_received_check && correct_char_received_check)
    return false;
  else
    return true;
}
}
