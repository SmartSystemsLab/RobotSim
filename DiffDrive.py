'''
DiffDrive.py

This file extends the robot class to simulate a differential dirve robot.
''' 

import numpy as np
import math
import robots
import matplotlib.pyplot as plt
from matplotlib import rcParams

class DiffDrive(robots.Robot):
  '''
  A differential drive robot is a 2D mobile robot with two independently
  controlled wheels.  this class simulates such a robot.
  '''
  def __init__(self, id, b, r, m):
    super(DiffDrive, self).__init__(id)
    self.b = b
    self.r = r
    self.m = m
    self.x = np.matrix(np.zeros([5, 1]))
    self.x_est = np.matrix(np.zeros([5, 1]))
    self.M = np.matrix(np.identity(5)) * 0.01;
    self.N = np.matrix(np.identity(2)) * 0.01;
    self.mu_m = np.matrix(np.zeros([5, 1]))
    self.mu_n = np.matrix(np.zeros([2, 1]))
    self.P = np.matrix(np.zeros([5, 5]))

  def f(self, x, u):
    x_new = np.matrix(np.zeros([5, 1]))

    x_new[0, 0] = x[0, 0] + 0.5*(u[0, 0] + u[1, 0])*math.cos(x[2, 0])*self.del_t
    x_new[1, 0] = x[1, 0] + 0.5*(u[0, 0] + u[1, 0])*math.sin(x[2, 0])*self.del_t
    x_new[2, 0] = x[2, 0] + self.del_t*(u[0, 0] - u[1, 0])/self.b
    x_new[3, 0] = x[3, 0] + u[0, 0]*self.del_t
    x_new[4, 0] = x[4, 0] + u[1, 0]*self.del_t

    if x_new[2, 0] > math.pi:
      x_new[2, 0] -= 2*math.pi
    elif x_new[2, 0] < -math.pi:
      x_new[2, 0] += 2*math.pi

    return x_new

  def h(self, x):
    z = np.matrix(np.zeros([2, 1]))

    z[0, 0] = x[3, 0]
    z[1, 0] = x[4, 0]

    return z

  def H(self, x):
    return np.matrix([[0, 0, 0, 1, 0], [0, 0, 0, 0, 1]])

  def draw3(self, ax, R):
    circle = np.matrix(np.zeros([3, 30]))
    line = np.matrix(np.zeros([3, 2]))

    for i in range(30):
      circle[0, i] = self.x[0, 0] + self.r*math.cos(i*2.0*math.pi/30.0)
      circle[1, i] = self.x[1, 0] + self.r*math.sin(i*2.0*math.pi/30.0)

      circle[:, i] = R * circle[:, i]

    line[0:2, 0] = self.x[0:2, 0]
    line[0:2, 1] = self.x[0:2, 0] + np.matrix([[self.r*math.cos(self.x[2, 0])], [self.r*math.sin(self.x[2, 0])]])

    line[:, 0] = R*line[:, 0]
    line[:, 1] = R*line[:, 1]

    ax.plot(circle[0, :], circle[1, :], circle[2, :], 'g')
    ax.plot(line[0, :], line[1, :], line[2, :], 'g')

  def draw(self, ax):
    circle = np.matrix(np.zeros([3, 30]))
    line = np.matrix(np.zeros([3, 2]))

    for i in range(30):
      circle[0, i] = self.x[0, 0] + self.r*math.cos(i*2.0*math.pi/30.0)
      circle[1, i] = self.x[1, 0] + self.r*math.sin(i*2.0*math.pi/30.0)

    line[0:2, 0] = self.x[0:2, 0]
    line[0:2, 1] = self.x[0:2, 0] + np.matrix([[self.r*math.cos(self.x[2, 0])], [self.r*math.sin(self.x[2, 0])]])

    ax.plot(circle[0, :], circle[1, :], 'g')
    ax.plot(line[0, :], line[1, :], 'g')

  def get_r2(self):
    return np.matrix([[1.0, 0.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0, 0.0]])*self.x

  def get_r3(self):
    return np.matrix([[1.0, 0.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0]])*self.x

if __name__ == '__main__':
  print 'Testing diffy drive robot sim'
  print 'setting up robot...'
  b = 1/math.pi
  rob = DiffDrive(0, b, 0.05, 1)

  print 'setting up simulation...'
  t_0 = 0
  t_f = 4.25
  n = (t_f - t_0)/rob.del_t + 2
  t = np.linspace(t_0, t_f, n)
  u1 = np.matrix([[1], [-1]])
  u2 = np.matrix([[1], [1]])
  u3 = np.matrix([[-1], [1]])
  x = [[], [], [], [], []]
  z = [[], []]

  x[0].append(rob.get_state()[0, 0])
  x[1].append(rob.get_state()[1, 0])
  x[2].append(rob.get_state()[2, 0])
  x[3].append(rob.get_state()[3, 0])
  x[4].append(rob.get_state()[4, 0])

  z[0].append(rob.get_sensor_reading()[0, 0])
  z[1].append(rob.get_sensor_reading()[1, 0])

  print 'running sim...'

  for i in range(int(n-1)):
    if t[i] <= 0.25:
      rob.apply_input(u1)
    elif t[i] < 1.25:
      rob.apply_input(u2)
    elif t[i] < 1.625:
      rob.apply_input(u3)
    elif t[i] < 2.125:
      rob.apply_input(u2)
    elif t[i] < 2.375:
      rob.apply_input(u1)
    elif t[i] < 2.875:
      rob.apply_input(u2)
    elif t[i] < 3.25:
      rob.apply_input(u3)
    else:
      rob.apply_input(u2)
    x_now = rob.get_state()
    z_now = rob.get_sensor_reading()

    x[0].append(x_now[0, 0])
    x[1].append(x_now[1, 0])
    x[2].append(x_now[2, 0])
    x[3].append(x_now[3, 0])
    x[4].append(x_now[4, 0])

    z[0].append(z_now[0, 0])
    z[1].append(z_now[1, 0])

    #fig = plt.figure(1)
    #ax = fig.gca()
    #rob.draw(ax)
    #plt.show()
    print i

  print 'plotting results'

  plt.figure(1)
  plt.plot(x[0], x[1])
  plt.ylabel('ylabel')
  plt.xlabel('xlabel')
  plt.title('I dont know what to call this because there are no comments')
  plt.figure(2)
  plt.plot(t, x[2])
  plt.ylabel('ylabel')
  plt.xlabel('xlabel')
  plt.title('I dont know what to call this because there are no comments (1)')
  plt.show()
