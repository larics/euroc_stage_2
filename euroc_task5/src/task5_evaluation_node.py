#! /usr/bin/python

import math

import rospy

from mav_msgs.msg import Status
from geometry_msgs.msg import TransformStamped
from scan_rectangle import ScanRectangle

from rotations import *


# The StatusSubscriber subscribes to the MAV's status topic. It is therefore
# able to determine wheter the MAV's motors are running or not. The motors's
# status is used to stop and start the timing in the evaluation of Task 5.
class StatusSubscriber:
  subscriber = None
  start_time = rospy.Time()
  uptime = rospy.Time()
  are_motors_running = False
  is_flight_finished = False

  def __init__(self):
    TOPIC_NAME = 'status'
    MESSAGE_TYPE = Status
    self.subscriber = rospy.Subscriber(TOPIC_NAME, MESSAGE_TYPE, self.SubscriberCallback)

  def AreMotorsRunning(self):
    return self.are_motors_running

  def IsFlightFinished(self):
    return self.is_flight_finished

  def GetUptime(self):
    return self.uptime.to_sec()

  def SubscriberCallback(self, input):
    are_motors_running = (input.motor_status == 'running')
    if (are_motors_running and not self.are_motors_running):
      # Motors were turned on.
      print 'Start evaluation.'
      self.start_time = input.header.stamp
      self.are_motors_running = are_motors_running

    elif (not are_motors_running and self.are_motors_running):
      # Motors were turned off.
      print 'Stop evaluation.'
      self.uptime = input.header.stamp - self.start_time
      self.are_motors_running = are_motors_running
      self.is_flight_finished = True

    else:
      # Update flight time.
      self.uptime = input.header.stamp - self.start_time


# The RectanglePoseSubscriber class subscribes to a ground truth rectangle pose
# topic, e.g., the rectangle pose returned by the Vicon motion capture system.
# The GetPose() member function returns the pose of the rectangle.
class RectanglePoseSubscriber:
  subscriber = None
  message_counter = 0
  translation = [0.0, 0.0, 0.0]         # W_p_WA; 3-vector (m).
  rotation = [0.0, 0.0, 0.0, 1.0]       # q_WA; Quaternion.

  def __init__(self):
    pass

  def SubscriberCallback(self, input):
    W_p_WA = input.transform.translation
    q_WA = input.transform.rotation
    self.translation = [W_p_WA.x, W_p_WA.y, W_p_WA.z]
    self.rotation = [q_WA.x, q_WA.y, q_WA.z, q_WA.w]

    self.message_counter += 1

  def GetPose(self):
    # Subscribe.
    TOPIC_NAME = 'vrpn_client_rec/estimated_transform'
    MESSAGE_TYPE = TransformStamped
    self.subscriber = rospy.Subscriber(TOPIC_NAME, MESSAGE_TYPE, self.SubscriberCallback)

    # Get single pose measurement.
    while self.message_counter < 1:
      if rospy.is_shutdown():
        print '\nCould not get rectangle pose message.\n'
        break

    # Unsubscribe.
    self.subscriber.unregister()

    return self.translation, self.rotation


# The Evaluator class subscribes to a ground truth pose topic of the MAV. On the
# basis of the pose messages, the Evaluator class computes:
# - the time needed for the task (by the use of its status_subscriber member)
# - the rectangle coverage (by the use of its scan_rectangle member)
# - the RMSE of the MAV's distance to the rectangle
# - the corrected time (time/coverage).
class Evaluator:
  status_subscriber = None;
  scan_rectangle = None;
  vicon_subscriber = None;
  vicon_message_seq = 0

  # Member variables related to the mean squared distance error metric.
  desired_distance = 0.0
  summed_squared_error = 0.0
  vicon_callback_counter = 0

  # Transformation from Vicon frame to the rectangle's frame.
  rectangle_translation = []    # W_p_WA.
  rectangle_rotation = []       # q_WA.
  rectangle_frame_width = 0.0

  def __init__(self, desired_distance, status_subscriber,
      scan_rectangle, rectangle_translation, rectangle_rotation, frame_width):
    self.desired_distance = desired_distance
    self.status_subscriber = status_subscriber
    self.scan_rectangle = scan_rectangle
    self.rectangle_translation = rectangle_translation
    self.rectangle_rotation = rectangle_rotation
    self.rectangle_frame_width = frame_width
    self.SubscribeToVicon()


  # Getter methods to query the metrics
  def GetDistanceRmse(self):
    if self.vicon_callback_counter == 0:
      mse = 0.0
    else:
      mse = self.summed_squared_error / self.vicon_callback_counter
    rmse = math.sqrt(mse)
    return rmse

  def GetUptime(self):
    return self.status_subscriber.GetUptime()

  def GetRectangleCoverage(self):
    return self.scan_rectangle.GetCoverage()

  def PrintResults(self):
    coverage = self.GetRectangleCoverage()
    print '\n--------------------------------------------'
    print 'Time:', self.GetUptime(), 's'
    print 'Rectangle coverage:', coverage
    print '--------------------------------------------'
    print 'Rectangle distance RMSE:', self.GetDistanceRmse(), 'm'
    if coverage == 0.0:
      print 'Corrected time: not defined for coverage = 0'
    else:
      print 'Corrected time:', self.GetUptime()/coverage, 's'
    print '--------------------------------------------\n'

  # Calls all the evaluation methods which are triggered by the vicon measurement.
  def ViconCallback(self, input):
    # Verify that no message was lost.
    current_seq = input.header.seq
    if (self.vicon_message_seq == 0) or (self.vicon_message_seq + 1 == current_seq):
      pass
    else:
      rospy.logwarn('Lost Vicon pose message. Current seq = %s; previous seq = %s.',
          current_seq, self.vicon_message_seq)

    self.vicon_message_seq = current_seq

    if not self.status_subscriber.AreMotorsRunning():
      return

    # Get all the transformations to compute the MAV's position and sensor
    # direction in the rectangle's coordinate frame.

    # Transformation from Vicon (W) to MAV (B).
    p = input.transform.translation
    q = input.transform.rotation
    W_p_WB = [p.x, p.y, p.z]
    q_WB = [q.x, q.y, q.z, q.w]

    # Transformation from Vicon (W) to rectangle (A).
    W_p_WA = self.rectangle_translation
    q_WA = self.rectangle_rotation

    # Transformation to compensate for the rectangle frame width.
    A_p_offset = [self.rectangle_frame_width, self.rectangle_frame_width, 0.0]

    # Vectors in MAV coordinate frame.
    B_p_BS = [0.0, 0.0, 0.0]    # CoG in MAV coordinate frame.
    B_d = [1.0, 0.0, 0.0]       # Sensor direction in MAV coordinate frame.

    # Compute MAV postion in rectangle's frame.
    W_p_WS = TransformVector(W_p_WB, q_WB, B_p_BS)
    A_p_AS_offset = InverseTransformVector(W_p_WA, q_WA, W_p_WS)
    A_p_AS = InverseTranslateVector(A_p_offset, A_p_AS_offset)

    # Compute sensor direction in rectangle's frame.
    W_d = RotateVector(q_WB, B_d)
    A_d = InverseRotateVector(q_WA, W_d)

    # Update rectangle coverage.
    sensor_pos = A_p_AS
    sensor_dir = A_d
    is_scanning = self.scan_rectangle.UpdateCoverageCellsFaster(sensor_pos, sensor_dir)

    # Update counter and compute summed squared error.
    if is_scanning:
      self.vicon_callback_counter += 1
      self.summed_squared_error += (self.desired_distance - A_p_AS[2])**2

  def SubscribeToVicon(self):
    TOPIC_NAME = 'vrpn_client/estimated_transform'
    MESSAGE_TYPE = TransformStamped
    self.vicon_subscriber = rospy.Subscriber(
        TOPIC_NAME, MESSAGE_TYPE, self.ViconCallback, queue_size=10)


def main():
  rospy.init_node('task5_evaluation_node', anonymous=True)

  # Get parameters from launch file.
  rectangle_width = rospy.get_param('~rectangle_width')
  rectangle_height = rospy.get_param('~rectangle_height')
  rectangle_frame_width = rospy.get_param('~frame_width')
  cell_size = rospy.get_param('~cell_size')

  # Task5-related constants.
  BEAM_ANGLE_IN_DEGREES = 10
  SENSOR_RANGE = 2.0
  DESIRED_DISTANCE = 1.5

  wx = rectangle_width - 2*rectangle_frame_width
  wy = rectangle_height - 2*rectangle_frame_width
  num_cells_x = int(wx/cell_size)
  num_cells_y = int(wy/cell_size)

  # Get pose of rectangle.
  rectangle_pose_subscriber = RectanglePoseSubscriber()
  W_p_WA = []
  q_WA = []
  W_p_WA, q_WA = rectangle_pose_subscriber.GetPose()
  print 'Rectangle translation:', W_p_WA
  print 'Rectangle rotation:', q_WA

  # Instanciate rectangle.
  scan_rectangle = ScanRectangle(wx, wy, num_cells_x, num_cells_y,
      BEAM_ANGLE_IN_DEGREES, SENSOR_RANGE)

  status_subscriber = StatusSubscriber()
  evaluator = Evaluator(DESIRED_DISTANCE, status_subscriber,
      scan_rectangle, W_p_WA, q_WA, rectangle_frame_width)


  while not (rospy.is_shutdown() or status_subscriber.IsFlightFinished()):    
    scan_rectangle.ShowPlot()

  evaluator.PrintResults()



if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
