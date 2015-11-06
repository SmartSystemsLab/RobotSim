
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
  plt.figure(2)
  plt.plot(t, x[2])
  plt.show()
