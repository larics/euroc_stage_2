#include <safe_joystick_nav/safe_joystick_nav.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "safe_joystick_nav_node");

  ros::NodeHandle nh, private_nh("~");

  ROS_INFO("Started Joystick Nav Node.");

  safe_joystick_nav::SafeJoystickNav SafeJoystickNav(nh, private_nh);

  ros::spin();

  return 0;
}
