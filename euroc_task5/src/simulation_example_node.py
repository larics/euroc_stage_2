#! /usr/bin/python

# The evaluation node of Task 5 subscribes to the 'status' topic of the MAV
# in order to determine the MAV's flight time. This node replicates the
# MAV's status message publisher such that the evaluation node can easily be
# used with the RotorS simulation.

import math

import rospy

from mav_msgs.msg import Status
from geometry_msgs.msg import TransformStamped


def main():
  rospy.init_node('simulation_example_node', anonymous=True)

  pub = rospy.Publisher('status', Status, queue_size=10)

  # Send motor_status = 'stopped'. The evaluation does not yet start.
  status = Status()
  status.header.stamp = rospy.Time.now()
  status.motor_status = 'stopped'
  pub.publish(status)

  # Run the evaluation for 60 seconds, i.e., send motor_status = 'running'
  # for 60 seconds.
  max_time_in_seconds = 60.0
  rate_in_hz = 10
  rate = rospy.Rate(rate_in_hz)
  i = 0
  while not rospy.is_shutdown():
    if float(i)/rate_in_hz > max_time_in_seconds:
      break
    status.header.stamp = rospy.Time.now()
    status.motor_status = 'running'
    pub.publish(status)
    rate.sleep()
    i += 1

  # Stop evaluation by sending motor_status = 'stopped'.
  status.header.stamp = rospy.Time.now()
  status.motor_status = 'stopped'
  pub.publish(status)

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
