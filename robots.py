'''
robots.py

This module contains classes for simulating various robots with internal controller
linearization.  These robots have internal models and are built to be plugged into
other simulations.
'''

import numpy as np
import math

class Robot(object):
  '''
  This is a parent class for all robots.  To create a new robot, make it a child class of this
  robot and overload the soecified functions.
  '''
  def __init__(self, id):
    self.id = id
    self.use_noise = False
    self.use_kalman = False
    self.M = np.matrix([1])
    self.N = np.matrix([1])
    self.mu_m = [0]
    self.mu_n = [0]
    self.x = np.matrix([0])
    self.del_t = 0.001
    self.del_x = np.matrix([0.01])
    self.x_est = np.matrix([0])
    self.P = np.matrix([0])

  def set_N(self, N):
    self.N = N
    n = math.sqrt(N.size)
    self.mu_n = []
    for i in range(n):
      self.mu_n.append(0)

  def set_M(self, M):
    self.M = M
    m = math.sqrt(M.size)
    self.mu_m = []
    for i in range(m):
      self.mu_m.append(0)

  def set_state(self, x):
    self.x = x

  def set_state_certainty(self, P):
    self.P = P

  def set_noise_use(self, use_noise):
    self.use_noise = use_noise

  def set_kalman_use(self, use_kalman):
    self.use_kalman = use_kalman

  def set_time_step(self, del_t):
    self.del_t = del_t

  def set_x_diff(self, del_x):
    self.del_x = del_x

  def set_state_estimate(self, x_est):
    self.x_est = x_est

  def f(self, x, u):
    return x + u

  def h(self, x):
    return x

  def F(self, x, u):
    n = self.x.size
    F = np.identity(n)
    for i in range(n):
      x_off = x
      x_off[i, 0] += self.del_x[i, 0]
      f1 = self.f(x, u)
      f2 = self.f(x_off, u)
      F[:, i] = (f2 - f1)/self.del_x[i, 0]
    return F

  def H(self, x):
    n = self.x.size
    m = self.h(x).size
    H = np.matrix(np.zeros([m, n]))
    for i in range(n):
      x_off = x
      x_off[i, 0] += self.del_x[i, 0]
      h1 = self.h(x)
      h2 = self.h(x_off)
      H[:, i] = (h2 - h1)/self.del_x[i, 0]
    return H

  def apply_input(self, u):
    self.x = self.f(self.x, u)

    if self.use_kalman:
      F = self.F(self.x_est, u)
      z = self.get_sensor_reading()
      self.x_est = self.f(self.x_est, u)
      self.P = F*self.P*F.transpose() + self.M
      H = self.H(self.x_est)
      y = z - self.h(self.x_est)
      S = H*self.P*H.transpose() + self.N
      K = self.P*H*np.linalg.inv(S)
      self.x_est += K*y
      self.P -= K*H*self.P
      return self.x_est
    else:
      return self.x

  def get_sensor_reading(self):
    if self.use_noise or self.use_kalman:
      return self.h(self.x) + np.matrix(np.random.multivariate_normal(self.mu_n, self.N)).transpose()
    else:
      return self.h(self.x)

  def get_state(self):
    return self.x

  def get_state_est(self):
    return self.x_est
