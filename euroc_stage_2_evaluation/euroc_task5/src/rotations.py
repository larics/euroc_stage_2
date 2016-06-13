#! /usr/bin/python

import numpy as np

def QuaternionToRotationMatrix(q):
  qx = q[0]
  qy = q[1]
  qz = q[2]
  qw = q[3]
  skew = np.matrix([[0.0, -qz, qy], [qz, 0.0, -qx], [-qy, qx, 0.0]])
  return np.eye(3) + 2.0*skew*skew + 2.0*qw*skew

# Translate vector. Returns v + p, e.g., v_WX = p_WB + v_BX.
def TranslateVector(p, v):
  v_out = [0.0] * 3
  v_out[0] = p[0] + v[0]
  v_out[1] = p[1] + v[1]
  v_out[2] = p[2] + v[2]
  return v_out

# Inverse of translation. Returns v - p, e.g., v_BX = -p_WB + v_WX.
def InverseTranslateVector(p, v):
  v_out = [0.0] * 3
  v_out[0] = v[0] - p[0]
  v_out[1] = v[1] - p[1]
  v_out[2] = v[2] - p[2]
  return v_out

# Rotate vector. Returns R(q) * v, e.g., W_v = R(q_WB) * B_v.
def RotateVector(q, v):
  R = QuaternionToRotationMatrix(q)
  v_out_np = R * np.matrix(v).transpose()

  # Copy the result to a list and return.
  v_out = [0.0] * 3
  v_out[0] = v_out_np.item(0)
  v_out[1] = v_out_np.item(1)
  v_out[2] = v_out_np.item(2)
  return v_out

# Rotate vector by inverse rotation. Returns R(q)^(-1) * v,
# e.g., B_v = R(q_WB)^(-1) * W_v.
def InverseRotateVector(q, v):
  q_inverse = [q[0], q[1], q[2], -q[3]]
  return RotateVector(q_inverse, v)

# Rotate and translate vector. Returns R(q) * v + p,
# e.g., W_v_WX = R(q_WB) * B_v_BX + W_p_WB.
def TransformVector(p, q, v):
  u = RotateVector(q, v)
  return TranslateVector(p, u)

# Inverse of rotation and translation. Returns R(q)^(-1) * (v - p),
# e.g., B_v_BX = R(q_WB)^(-1) * (W_v_WX - W_p_WB).
def InverseTransformVector(p, q, v):
  u = InverseTranslateVector(p, v)
  return InverseRotateVector(q, u)
