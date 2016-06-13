#! /usr/bin/python

import unittest
import sys
import os
sys.path.append(os.path.abspath("../src"))

import numpy as np
import random

from rotations import *
from scan_rectangle import ScanRectangle
from task5_evaluation_node import Evaluator
from task5_evaluation_node import StatusSubscriber

from geometry_msgs.msg import TransformStamped


def EuclideanDistance(vec1, vec2):
  v1 = np.array(vec1)
  v2 = np.array(vec2)
  return np.linalg.norm(v1 - v2)

class TestRotationFunctions(unittest.TestCase):

  # Test if the vectors remain unchanged under the identity transform.
  def test_identity(self):
    t = [0.0, 0.0, 0.0]
    q = [0.0, 0.0, 0.0, 1.0]
    v_in = [1.0, 2.0, 3.0]

    v_out = TransformVector(t, q, v_in)
    self.assertLess(EuclideanDistance(v_in, v_out), 1.0e-10)

    v_out = RotateVector(q, v_in)
    self.assertLess(EuclideanDistance(v_in, v_out), 1.0e-10)

    v_out = TranslateVector(t, v_in)
    self.assertLess(EuclideanDistance(v_in, v_out), 1.0e-10)

  # Tests if the functions shift the vectors for the correct distance and tests
  # for consistency between different methods
  def test_shift(self):
    t = [1.0, 2.0, 3.0]
    q = [0.0, 0.0, 0.0, 1.0]
    v_in = [1.0, 2.0, 3.0]
    distance = np.linalg.norm(np.array(t))

    v_out1 = TransformVector(t, q, v_in)
    distance_diff = distance - EuclideanDistance(v_in, v_out1)
    self.assertLess(np.abs(distance_diff), 1.0e-10)

    v_out2 = TranslateVector(t, v_in)
    distance_diff = distance - EuclideanDistance(v_in, v_out2)
    self.assertLess(np.abs(distance_diff), 1.0e-10)

    # Test consistency between Transform() and Translate() function.
    self.assertLess(EuclideanDistance(v_out1, v_out2), 1.0e-10)

  # Test if the functions do not change the vector norm for rotations and tests
  # for consistency between different methods
  def test_rotation(self):
    t = [0.0, 0.0, 0.0]
    q = [1.0, 2.0, 3.0, 4.0]
    q = [element/np.linalg.norm(np.array(q)) for element in q]  # Normalize.
    v_in = [1.0, 2.0, 3.0]
    v_in_norm = np.linalg.norm(np.array(v_in))

    v_out1 = TransformVector(t, q, v_in)
    norm_diff = np.linalg.norm(np.array(v_out1)) - v_in_norm
    self.assertLess(np.abs(norm_diff), 1.0e-10)

    v_out2 = RotateVector(q, v_in)
    norm_diff = np.linalg.norm(np.array(v_out2)) - v_in_norm
    self.assertLess(np.abs(norm_diff), 1.0e-10)

    # Test consistency between Transform() and Rotate() function.
    self.assertLess(EuclideanDistance(v_out1, v_out2), 1.0e-10)

  # Test if the inverse function value of the function value equals the input value.
  def test_inverse_Functions(self):
    t = [3.1, 2.2, 1.3]
    q = [1.0, 2.0, 3.0, 4.0]
    q = [element/np.linalg.norm(np.array(q)) for element in q]  # Normalize.
    v_in = [1.0, 2.0, 3.0]

    v_in2 = TransformVector(t, q, v_in)
    v_out = InverseTransformVector(t, q, v_in2)
    self.assertLess(EuclideanDistance(v_in, v_out), 1.0e-10)

    v_in2 = RotateVector(q, v_in)
    v_out = InverseRotateVector(q, v_in2)
    self.assertLess(EuclideanDistance(v_in, v_out), 1.0e-10)

    v_in2 = TranslateVector(t, v_in)
    v_out = InverseTranslateVector(t, v_in2)
    self.assertLess(EuclideanDistance(v_in, v_out), 1.0e-10)


class TestScanRectangle(unittest.TestCase):
  width = 1.0
  height = 2.0
  sensor_range = 2.0
  beam_angle_in_degrees = 10.0
  # Preferably keep nx and ny odd. Otherwise for some tests, where the
  # intersection of the beam and the rectangle is very narrow, zero coverage
  # might be returned (depending on the number of cells) and the tests will fail. 
  nx = 101
  ny = 201

  def GetRectangle(self):
    return ScanRectangle(self.width, self.height, self.nx, self.ny,
                         self.beam_angle_in_degrees, self.sensor_range)

  # Updates the coverage cells of a ScanRectangle object while performing
  # additional tests.
  def UpdateCoverageCells(self, rectangle, sensor_pos, sensor_direction):
    # Temporary store the member variables related to the coverage of the
    # rectangle. Assumes that num_covered and coverage_cells are the only
    # member variables related to the rectangle's coverage.
    n = rectangle.num_covered
    cells = np.array(rectangle.coverage_cells)

    # Update the rectangle's coverage by the first method.
    is_scanning1 = rectangle.UpdateCoverageCells(sensor_pos, sensor_direction)
    coverage1 = rectangle.GetCoverage()
    # Assert correct counting.
    self.assertEqual(rectangle.num_covered, np.sum(rectangle.coverage_cells))
    # Undo the UpdateCoverageCells() call.
    rectangle.num_covered = n
    rectangle.coverage_cells = np.array(cells)

    # Update the rectangle's coverage by the second method.
    is_scanning2 = rectangle.UpdateCoverageCellsFaster(
        sensor_pos, sensor_direction)
    coverage2 = rectangle.GetCoverage()
    # Assert correct counting.
    self.assertEqual(rectangle.num_covered, np.sum(rectangle.coverage_cells))

    # Assert equality of both update methods.
    self.assertEqual(coverage1, coverage2)
    self.assertEqual(is_scanning1, is_scanning2)

    return is_scanning1

  def test_initial_coverage(self):
    rectangle = self.GetRectangle()
    self.assertEqual(rectangle.GetCoverage(), 0.0)

  def test_full_coverage(self):
    rectangle = self.GetRectangle()
    distance = 1.5
    radius = distance * np.tan(0.5 * rectangle.beam_angle)
    step_size = radius * np.sqrt(2.0)

    # The test assumes that the sensor range is not exceeded. Check if
    # this assumption is violated. If the following assertion fails, decrease
    # the distance variable until you get a valid test.
    max_distance_squared = distance**2 + (0.5*step_size)**2 + (0.5*step_size)**2 
    self.assertLess(max_distance_squared, self.sensor_range**2)
  
    # Number of steps in x and y-direction which are necessary for full coverage.
    nx = np.ceil(self.width/step_size)
    ny = np.ceil(self.height/step_size)

    i = 1
    x = step_size/2.0
    while x < self.width + step_size/2.0:
      j = 1
      y = step_size/2.0
      while y < self.height + step_size/2.0:
        position = [x, y, distance]
        direction = [0.0, 0.0, -1.0]
        is_scanning = self.UpdateCoverageCells(rectangle, position, direction)
        self.assertTrue(is_scanning)
        expected_min_coverage = float((i-1)*ny + j)/(nx*ny)
        coverage_diff = rectangle.GetCoverage() - expected_min_coverage
        self.assertTrue(coverage_diff >= 0.0)

        # rectangle.ShowPlot()
        j += 1
        y += step_size

      i += 1
      x += step_size

    # Expect full coverage.
    coverage_diff = rectangle.GetCoverage() - 1.0
    self.assertTrue(np.abs(coverage_diff) < 1.0e-10)

  def test_exceed_max_range_scan(self):
    pos = [0.5*self.width, 0.5*self.height, self.sensor_range]
    direction = [0.0, 0.0, -1.0]

    # For a position outside of the sensor range, expect coverage = 0.
    pos_far = list(pos)
    pos_far[2] += 0.1
    rectangle = self.GetRectangle()
    is_scanning = self.UpdateCoverageCells(rectangle, pos_far, direction)
    self.assertFalse(is_scanning)
    self.assertEqual(rectangle.GetCoverage(), 0.0)

    # For a position within the sensor range, expect coverage > 0.
    pos_closer = list(pos)
    pos_closer[2] -= 0.1
    rectangle = self.GetRectangle()
    is_scanning = self.UpdateCoverageCells(rectangle, pos_closer, direction)
    self.assertTrue(is_scanning)
    self.assertGreater(rectangle.GetCoverage(), 1.0e-5)

  def test_backside_scan(self):
    rectangle = self.GetRectangle()
    position = [0.5*self.width, 0.5*self.height, -1.0]
    direction = [0.0, 0.0, 1.0]
    is_scanning = self.UpdateCoverageCells(rectangle, position, direction)
    self.assertFalse(is_scanning)
    self.assertEqual(rectangle.GetCoverage(), 0.0)

  def test_facing_away_scan(self):
    rectangle = self.GetRectangle()

    # Test sensor direction.
    a = np.tan(0.5 * rectangle.beam_angle)
    d1 = [0.0, 0.0, 1.0]
    d2 = [0.0, 1.0, a]
    d3 = [1.0, 0.0, a]
    d4 = [0.5*np.sqrt(2), 0.5*np.sqrt(2), a]
    d_list = [d1, d2, d3, d4]

    # Test sensor positions.
    p1 = [0.5*self.width, 0.5*self.height, 1.0]   # 1m distance to rectangle.
    p2 = [0.5*self.width, 0.5*self.height, 0.01]  # Very close to rectangle.
    p_list = [p1, p2]

    for d in d_list:
      for p in p_list:
        is_scanning = self.UpdateCoverageCells(rectangle, p, d)
        self.assertFalse(is_scanning)
        self.assertEqual(rectangle.GetCoverage(), 0.0)

  def test_hyperbola_scan(self):
    # Test sensor direction.
    d1 = [1.0, 0.0, 0.0]
    d2 = [-1.0, 0.0, 0.0]
    d3 = [0.0, 1.0, 0.0]
    d4 = [0.0, -1.0, 0.0]
    d5 = [1.0, 1.0, 0.0]
    d6 = [-1.0, -1.0, 0.0]
    d_list = [d1, d2, d3, d4, d5, d6]

    p1 = [0.5*self.width, 0.5*self.height, 0.01]  # Very close to rectangle.
    p2 = [0.5*self.width, 0.5*self.height, 1.0e-10]  # Very, very close to rectangle.
    p_list = [p1, p2]

    for d in d_list:
      for p in p_list:
        rectangle = self.GetRectangle()
        is_scanning = self.UpdateCoverageCells(rectangle, p, d)
        self.assertTrue(is_scanning)
        self.assertGreater(rectangle.GetCoverage(), 0.001)
        # rectangle.ShowPlot()

  def test_parabola_scan(self):
    rectangle = self.GetRectangle()

    # Test sensor direction.
    a = np.tan(0.5 * rectangle.beam_angle)
    d1 = [1.0, 0.0, -a]
    d2 = [-1.0, 0.0, -a]
    d3 = [0.0, 1.0, -a]
    d4 = [0.0, -1.0, -a]
    d_list = [d1, d2, d3, d4]

    # Test sensor positions.
    p1 = [0.5*self.width, 0.5*self.height, 0.01]  # Very close to rectangle.
    p2 = [0.5*self.width, 0.5*self.height, 1.0e-10]  # Very, very close to rectangle.
    p_list = [p1, p2]

    for d in d_list:
      for p in p_list:
        rectangle = self.GetRectangle()
        is_scanning = self.UpdateCoverageCells(rectangle, p, d)
        self.assertTrue(is_scanning)
        self.assertGreater(rectangle.GetCoverage(), 0.0)
        # rectangle.ShowPlot()


class TestScanDistanceRmse(unittest.TestCase):

  scan_rectangle = None
  status_subscriber = None
  evaluator = None
  pose_message_header_Seq = 0
  rectangle_width = 1.0
  rectangle_height = 2.0
  desired_distance = 1.5
  sensor_range = 2.0
  beam_angle_in_degrees = 10.0

  def setUp(self):
    # Initialize the scan rectangle
    nx = 101
    ny = 201
    self.scan_rectangle = ScanRectangle(
        self.rectangle_width, self.rectangle_height,
        nx, ny, self.beam_angle_in_degrees, self.sensor_range)

    # Initialize the status subscriber. 'Turn on' the motors.
    self.status_subscriber = StatusSubscriber()
    self.status_subscriber.are_motors_running = True;

    # Set scan rectangle position such that the lower left corner is in the
    # world coordinate frame's origin and the normal is parallel to the world
    # coordinates frame's z-axis.
    rectangle_frame_width = 0.1
    W_p_WA = [-rectangle_frame_width, -rectangle_frame_width, 0.0]
    q_WA = [0.0, 0.0, 0.0, 1.0]

    self.evaluator = Evaluator(self.desired_distance, self.status_subscriber,
        self.scan_rectangle, W_p_WA, q_WA, rectangle_frame_width)

  # Returns a TransformStamped message. The input parameter are the desired
  # position of the MAV in world coordinates (W_p_WB) and the desired sensor
  # direction in world coordinates (W_d).
  def GetPoseMessage(self, position, direction):
    # Compute the quaternion of the MAV orientation
    W_d = direction / np.linalg.norm(direction)
    B_d = [1.0, 0.0, 0.0]

    angle = np.arccos(np.dot(B_d, W_d))
    axis = np.cross(B_d, W_d)
    q_WB = [axis[0] * np.sin(0.5*angle),
            axis[1] * np.sin(0.5*angle),
            axis[2] * np.sin(0.5*angle),
            1.0 * np.cos(0.5*angle)]

    # print 'angle: ', angle
    # print 'axis: ', axis
    # print 'quaternion: ', q_WB

    # Fill message.
    message = TransformStamped()
    message.header.seq = self.pose_message_header_Seq
    self.pose_message_header_Seq += 1

    W_p_WB = position
    message.transform.translation.x = W_p_WB[0]
    message.transform.translation.y = W_p_WB[1]
    message.transform.translation.z = W_p_WB[2]

    message.transform.rotation.x = q_WB[0]
    message.transform.rotation.y = q_WB[1]
    message.transform.rotation.z = q_WB[2]
    message.transform.rotation.w = q_WB[3]

    return message

  # Expect RMSE = 0.0 for scans with the exact desired distance.
  def test_zero_rmse(self):
    position = [0.5*self.rectangle_width,
                0.5*self.rectangle_height,
                self.desired_distance]
    direction = [0.0, 0.0, -1.0]
    num_samples = 50
    for i in range(0, num_samples):
      message = self.GetPoseMessage(position, direction)
      self.evaluator.ViconCallback(message)

    self.assertEqual(self.evaluator.GetDistanceRmse(), 0.0)
    self.assertEqual(self.evaluator.vicon_callback_counter, num_samples)

  # Expect RMSE = d for scans with a distance of desired_distance + d.
  def test_constant_rmse(self):
    offset = 0.1
    position = [0.5*self.rectangle_width,
                0.5*self.rectangle_height,
                self.desired_distance + offset]
    direction = [0.0, 0.0, -1.0]
    num_samples = 50
    for i in range(0, num_samples):
      message = self.GetPoseMessage(position, direction)
      self.evaluator.ViconCallback(message)

    diff = self.evaluator.GetDistanceRmse() - offset
    self.assertLess(np.abs(diff), 1.0e-10)
    self.assertEqual(self.evaluator.vicon_callback_counter, num_samples)

  # For randomly perturbed scan distances from desired_distanc, compare
  # the return value of Evaluator.GetDistanceRmse() to the manually computed
  # RMSE.
  def test_random_rmse(self):
    num_samples = 5000
    offset_list = [random.uniform(-0.1, 0.1) for _ in range (num_samples)]
    offset_rmse = np.linalg.norm(offset_list) / np.sqrt(num_samples)

    for i in range(0, num_samples):
      position = [0.5*self.rectangle_width,
                  0.5*self.rectangle_height,
                  self.desired_distance + offset_list[i]]
      direction = [0.0, 0.0, -1.0]
      message = self.GetPoseMessage(position, direction)
      self.evaluator.ViconCallback(message)

    # print 'Returned RMSE: ', self.evaluator.GetDistanceRmse()
    # print 'Expected RMSE', offset_rmse
    diff = self.evaluator.GetDistanceRmse() - offset_rmse
    self.assertLess(np.abs(diff), 1.0e-10)
    self.assertEqual(self.evaluator.vicon_callback_counter, num_samples)

  # Test the correct return of Evaluator.GetDistanceRmse() if some of the
  # pose messages are from non-scanning poses.
  def test_out_of_range_rmse(self):
    num_invalid_samples = 50*3
    num_valid_samples = 100

    # Scan from backside of rectangle.
    for i in range(0, num_invalid_samples/3):
      position = [0.5*self.rectangle_width,
                  0.5*self.rectangle_height,
                  - (1.0 + random.uniform(-0.1, 0.1))]
      direction = [0.0, 0.0, 1.0]
      message = self.GetPoseMessage(position, direction)
      self.evaluator.ViconCallback(message)

    # Exceed maximum range of sensor.
    for i in range(0, num_invalid_samples/3):
      position = [0.5*self.rectangle_width,
                  0.5*self.rectangle_height,
                  self.sensor_range + 0.01 + random.uniform(0.0, 1.0)]
      direction = [0.0, 0.0, -1.0]
      message = self.GetPoseMessage(position, direction)
      self.evaluator.ViconCallback(message)

    # Scan with facing away sensor.
    for i in range(0, num_invalid_samples/3):
      position = [0.5*self.rectangle_width,
                  0.5*self.rectangle_height,
                  self.desired_distance + random.uniform(-0.1, 0.1)]
      direction = [random.uniform(-0.5, 0.5),
                   random.uniform(-0.5, 0.5),
                   random.uniform(0.5, 1.0)]
      message = self.GetPoseMessage(position, direction)
      self.evaluator.ViconCallback(message)

    # Valid scans.
    offset_list = [random.uniform(-0.1, 0.1) for _ in range (num_valid_samples)]
    for i in range(0, num_valid_samples):
      position = [0.5*self.rectangle_width,
                  0.5*self.rectangle_height,
                  self.desired_distance + offset_list[i]]
      # Assert validity of the test.
      self.assertTrue(position[2] > 0.0 and position[2] < self.sensor_range)
      direction = [0.0, 0.0, -1.0]
      message = self.GetPoseMessage(position, direction)
      self.evaluator.ViconCallback(message)

    offset_rmse = np.linalg.norm(offset_list) / np.sqrt(num_valid_samples)
    # print 'Returned RMSE: ', self.evaluator.GetDistanceRmse()
    # print 'Expected RMSE', offset_rmse
    diff = self.evaluator.GetDistanceRmse() - offset_rmse
    self.assertLess(np.abs(diff), 1.0e-10)
    self.assertEqual(self.evaluator.vicon_callback_counter, num_valid_samples)



if __name__ == '__main__':
    unittest.main()
