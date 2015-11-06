'''
swarm.py

contains a class for extending the Robot class defined in robots.py to simulate
a swarm of robots

Authors:
Kyle Crandall (crandallk@gwu.edu)
'''

import numpy as np
import math
import robots

class Swarm:
  '''
  simulates a swarm of robots
  '''

  def __init__(self):
    self.members = []
    self.adj_mat = np.mat([[]])
    self.state_size = 0
    self.n = 0

  def add_member(self, member):
    self.members.append(member)
    self.n += 1
    self.state_size += member.x.shape[0]

  def remove_member(self, id):
    for i in range(len(self.members)):
      if self.members[i].id == id:
        self.n -= 1
        self.state_size -= self.members[i].x.shape[0]
        del self.members[i]
        return

  def get_swarm_state(self):
    x_s = np.mat(np.zeros((self.state_size, 1)))
    pos = 0
    for i in range(len(self.members)):
      x_s[pos:pos+self.members[i].x.shape[0], 0] = self.members[i].x
      pos += self.member[i].x.shape[0]
    return x_s

  def update_swarm(self, )
