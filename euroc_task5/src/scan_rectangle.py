#! /usr/bin/python

import numpy as np
import time
from matplotlib import pyplot as plt
from itertools import chain

# The ScanRectangle class stores, updates and plots the coverage of a rectangle
# by multiple scans from a virtual scanning sensor.

# The ScanRectangle's coordinate system:
# The origin of the coordinate system is in the lower left corner of the
# rectangle. The x-axis points to the right; the y-axis points upwards and
# the z-axis points parallel to the rectangle normal to the front side of the
# rectangle (i.e., the side from which the rectangle can be scanned).

class ScanRectangle:
  coverage_cells = None   # 2D array to keep track of covered grid cells.
  num_covered = 0         # Counter variables for covered cells.
  
  wx = 0.0                # Size of the rectangle in x-direction in meters.
  wy = 0.0                # Size of the rectangle in y-direction in meters.
  nx = 0                  # Number of grid cells in x-direction.
  ny = 0                  # Number of grid cells in y-direction.

  beam_angle = 0.0        # In radians.
  sensor_range = 0.0      # In meters.

  coverage_map = None     # Plot object.

  def __init__(self, wx, wy, nx, ny, beam_angle_in_degrees, sensor_range): 
    self.wx = wx
    self.wy = wy
    self.nx = nx
    self.ny = ny
    self.beam_angle = beam_angle_in_degrees*np.pi/180.0
    self.sensor_range = sensor_range
    self.coverage_cells = np.zeros((nx, ny), dtype=bool)

  def GetCoverage(self):
    return self.num_covered * (1.0/(self.nx*self.ny))

  # Updates the map coverage. Returns true if at least one cell of the 
  # coverage map is within the sensor's range and beam angle.
  def UpdateCoverageCells(self, sensor_pos, sensor_direction):
    # Boolean value to indicate whether the current MAV pose allows scanning
    # of the target or not. It is the return value of the function.
    is_scanning = False

    # Check for scanning form behind the rectangular scan area.
    if sensor_pos[2] <= 0.0:
      return is_scanning

    # Copy and normalize sensor direction vector.
    sensor_dir = list(sensor_direction)
    sensor_dir_sqrt_norm = sensor_dir[0]**2+sensor_dir[1]**2+sensor_dir[2]**2
    sensor_dir[0] *= (1.0 / np.sqrt(sensor_dir_sqrt_norm))
    sensor_dir[1] *= (1.0 / np.sqrt(sensor_dir_sqrt_norm))
    sensor_dir[2] *= (1.0 / np.sqrt(sensor_dir_sqrt_norm))

    # Preallocate lists.
    p_AC = [0.0] * 3
    p_SC = [0.0] * 3

    # Precomputations.
    dx = self.wx/self.nx
    dy = self.wy/self.ny
    sensor_range_squared = self.sensor_range**2
    min_cos = np.cos(0.5 * self.beam_angle)

    for i in range(0, self.nx):
      for j in range(0, self.ny):
        if (self.coverage_cells[i, j] and is_scanning):
          pass  # Cell (i, j) is allready covered.
        else:
          # Vector from rectangle origin (denoted A) to cell i,j (denoted C).
          p_AC[0] = (0.5 + i)*dx
          p_AC[1] = (0.5 + j)*dy
          # p_AC[2] = 0.0  allready initialized

          # Vector from sensor position (denoted S) to cell i,j (denoted C).
          p_SC[0] = p_AC[0] - sensor_pos[0]
          p_SC[1] = p_AC[1] - sensor_pos[1]
          p_SC[2] = p_AC[2] - sensor_pos[2]
          sensor_distance_squared = p_SC[0]**2 + p_SC[1]**2 + p_SC[2]**2

          if sensor_distance_squared <= sensor_range_squared:
            dot_product = (p_SC[0]*sensor_dir[0] +
                           p_SC[1]*sensor_dir[1] +
                           p_SC[2]*sensor_dir[2])
            cosinus = dot_product / np.sqrt(sensor_distance_squared)
            if cosinus >= min_cos:
              is_scanning = True
              if (not self.coverage_cells[i,j]):
                self.coverage_cells[i,j] = True
                self.num_covered += 1

    return is_scanning


  # Updates the map coverage. Returns true if at least one cell of the 
  # coverage map is within the sensor's range and beam angle.
  def UpdateCoverageCellsFaster(self, sensor_pos, sensor_direction):
    # Copy and normalize sensor direction vector.
    sensor_dir = list(sensor_direction)
    sensor_dir_sqrt_norm = sensor_dir[0]**2+sensor_dir[1]**2+sensor_dir[2]**2
    sensor_dir[0] *= (1.0 / np.sqrt(sensor_dir_sqrt_norm))
    sensor_dir[1] *= (1.0 / np.sqrt(sensor_dir_sqrt_norm))
    sensor_dir[2] *= (1.0 / np.sqrt(sensor_dir_sqrt_norm))

    # Boolean value to indicate whether the current MAV pose allows scanning
    # of the target or not. It is the return value of the function.
    is_scanning = False

    # Cannot exceed sensor range for scanning and cannot scan from the backside
    # of the rectangle.
    if (sensor_pos[2] > self.sensor_range or sensor_pos[2] <= 0):
      return is_scanning

    # Cannot scan with a sensor facing away from the rectangle (taking beam 
    # angle into account).
    if (np.arccos(-sensor_dir[2]) > 0.5*np.pi + 0.5*self.beam_angle):
      return is_scanning

    # Preallocate lists.
    p_AC = [0.0] * 3
    p_SC = [0.0] * 3

    # Precomputations.
    dx = self.wx/self.nx
    dy = self.wy/self.ny
    sensor_range_squared = self.sensor_range**2
    min_cos = np.cos(0.5 * self.beam_angle)

    # Direction along symmetry axis of the beam cone. ||d|| = 1.
    d = np.matrix([[sensor_dir[0]],[sensor_dir[1]],[sensor_dir[2]]])
    # Sensor positon.
    p = np.matrix([[sensor_pos[0]],[sensor_pos[1]],[sensor_pos[2]]])
    # Points x inside the beam cone satisfy x^T*H*x + h^T*x + g <= 0.
    # Compute H, h and g (Note: there is some vector geometry involved in
    # obtaining the formulae).
    beta = -(np.tan(0.5*self.beam_angle))**2
    H = ((1.0+beta)-2.0) * d * d.transpose() + np.eye(3)
    h = -2.0 * p.transpose() * H
    g = p.transpose() * H * p

    # For points on the rectangle, the z-component is zero. Compute coefficient
    # of the quadratic inequality in x and y.
    axx = H[0,0]
    axy = 2.0 * H[0,1]
    ayy = H[1,1]
    ax = h[0,0]
    ay = h[0,1]
    a0 = g[0,0]

    for i in range(0, self.nx):
      # Compute qadratic equation in y for z = 0 and x = (0.5 + i)*dx.
      x = (0.5 + i)*dx
      b2 = ayy;
      b1 = axy*x + ay
      b0 = axx*(x**2) + ax*x + a0
      discriminant = b1**2 - 4*b2*b0

      # Find iteration range for b2*y^2 + b1*y + b0 <= 0, i.e., the point
      # ((0.5 + i)*dx, y, 0.0) is in the beam cone.
      iteration_range = []

      if b2 == 0 and b1 == 0: # If the quadratic function is constant.
        if b0 <= 0:
          iteration_range = range(0, self.ny)
        else:
          pass  # iteration_range = []

      elif b2 == 0: # If the quadratic function is an affine function.
        y0 = -b0 / b1
        if b1 > 0:
          j0 = int(np.ceil(y0/dy - 0.5)) + 1  # +1 for robustness against numerical errors.
          j0 = min(max(0, j0), self.ny-1)
          iteration_range = range(0, j0+1)
        else: # b1 < 0
          j0 = int(np.floor(y0/dy - 0.5)) - 1
          j0 = min(max(0, j0), self.ny)
          iteration_range = range(j0, self.ny)

      # After handling constant and affine cases, the strictly quadratic cases
      # are handled.

      elif discriminant < 0:    # No real solution exists.
        pass  # iteration_range = []

      else: # discriminant >= 0, i.e., there are two real zeros y1 and y2
        y1 = (-b1 - np.sqrt(discriminant)) / (2.0 * b2)
        y2 = (-b1 + np.sqrt(discriminant)) / (2.0 * b2)

        if b2 > 0:  # Quadratic function in y is convex.
          j_lb = int(np.floor(y1/dy - 0.5)) - 1
          j_ub = int(np.ceil(y2/dy - 0.5)) + 1

          j_lb = min(max(0, j_lb), self.ny)
          j_ub = min(max(0, j_ub), self.ny-1)
          iteration_range = range(j_lb, j_ub+1)

        else: # b2 < 0; quadratic function in y is concave.
          j_lb = int(np.ceil(y1/dy - 0.5)) + 1
          j_ub = int(np.floor(y2/dy - 0.5)) - 1

          j_lb = min(max(0, j_lb), self.ny-1)
          j_ub = min(max(0, j_ub), self.ny)
          iteration_range = chain(range(0, j_lb+1), range(j_ub, self.ny))


      for j in iteration_range:
        if (self.coverage_cells[i, j] and is_scanning):
          pass  # Cell (i, j) is allready covered.
        else:
          # Vector from rectangle origin (denoted A) to cell i,j (denoted C).
          p_AC[0] = (0.5 + i)*dx
          p_AC[1] = (0.5 + j)*dy
          # p_AC[2] = 0.0  allready initialized

          # Vector from sensor position (denoted S) to cell i,j (denoted C).
          p_SC[0] = p_AC[0] - sensor_pos[0]
          p_SC[1] = p_AC[1] - sensor_pos[1]
          p_SC[2] = p_AC[2] - sensor_pos[2]
          sensor_distance_squared = p_SC[0]**2 + p_SC[1]**2 + p_SC[2]**2

          if sensor_distance_squared <= sensor_range_squared:
            dot_product = (p_SC[0]*sensor_dir[0] +
                           p_SC[1]*sensor_dir[1] +
                           p_SC[2]*sensor_dir[2])
            cosinus = dot_product / np.sqrt(sensor_distance_squared)
            if cosinus >= min_cos:
              is_scanning = True
              if (not self.coverage_cells[i,j]):
                self.coverage_cells[i,j] = True
                self.num_covered += 1

    return is_scanning

  def ShowPlot(self):
    if self.coverage_map == None:
      self.fig, ax = plt.subplots()
      self.fig.canvas.set_window_title('Rectangle Coverage')
      plt.ion()

      # By temporary changing the value of self.coverage_cells[0, 0], it is
      # ensured that coverage_cells contains both true and false values.
      # This way, imshow() properly sets the color map.
      value0 = self.coverage_cells[0, 0]
      self.coverage_cells[0, 0] = not self.coverage_cells[1, 0]

      self.coverage_map = ax.imshow(self.coverage_cells.transpose(),
          cmap=plt.cm.Blues, interpolation="nearest")

      self.coverage_cells[0, 0] = value0

      ax.set_xticks([])
      ax.set_yticks([])
      ax.invert_yaxis()
      plt.show()

    else:
      self.coverage_map.set_data(self.coverage_cells.transpose())
      self.fig.canvas.draw()



# Simple test program with visualization that uses the ScanRectangle class.
if __name__ == "__main__":
  num_cells_x = 100
  num_cells_y = 300
  rectangle_width = 0.5
  rectangle_height = 1.5
  beam_angle_in_degrees = 10.0
  sensor_range = 2.0

  scan_rectangle = ScanRectangle(rectangle_width, rectangle_height,
      num_cells_x, num_cells_y, beam_angle_in_degrees, sensor_range)

  sensor_pos = [0.5, 0.5, 1.5]
  sensor_dir = [0.0, 0.0, -1.0]

  scan_rectangle.ShowPlot()

  for i in range(0, 80):
    # Test trajectory.
    sensor_pos[1] += 0.05/4
    sensor_pos[2] += 0.03/4
    sensor_dir[0] -= 0.01/4

    t0 = time.time()
    scan_rectangle.UpdateCoverageCellsFaster(sensor_pos, sensor_dir)
    t1 = time.time()

    scan_rectangle.ShowPlot()
    t2 = time.time()
    print 'coverage: ', scan_rectangle.GetCoverage()
    print 'map update: ', t1 - t0, 's'
    print 'map display: ', t2 - t1, 's'

    time.sleep(0.01)


