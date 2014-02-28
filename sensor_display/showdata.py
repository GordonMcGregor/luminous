'''
 showdata.py

 Display analog data from Arduino using Python (matplotlib)
 
 Modified from an original available from electronut.in

 requires pyserial (pip install pyserial)
'''


import sys, serial
import numpy as np
from time import sleep
from collections import deque
from matplotlib import pyplot as plt

# class that holds analog data for N samples
class AnalogData:
  # constr
  def __init__(self, maxLen):
    self.ax = deque([0.0]*maxLen)
    self.ay = deque([0.0]*maxLen)
    self.maxLen = maxLen
    self.max_x = 0.1
    self.max_y = 0.1

  # ring buffer
  def addToBuf(self, buf, val):
    if len(buf) < self.maxLen:
      buf.append(val)
    else:
      buf.popleft()
      buf.append(val)

  # add data
  def add(self, data):
    assert(len(data) == 4)
    self.max_x = data[0]
    self.max_y = data[2]
    self.addToBuf(self.ax, (data[1]/self.max_x)*1023)
    self.addToBuf(self.ay, (data[3]/self.max_y)*1023)
    
# plot class
class AnalogPlot:
  # constr
  def __init__(self, analogData):
    # set plot to animated
    plt.ion() 
    self.axline, = plt.plot(analogData.ax)
    self.ayline, = plt.plot(analogData.ay)
    plt.ylim([0, 1023])

  # update plot
  def update(self, analogData):
    self.axline.set_ydata(analogData.ax)
    self.ayline.set_ydata(analogData.ay)
    plt.draw()

samples = []
# main() function
def main():
  # expects 1 arg - serial port string
  if(len(sys.argv) != 2):
    print 'Example usage: python showdata.py "/dev/tty.usbmodem411"'
    exit(1)

  strPort = sys.argv[1];

  print 'Using serial port', strPort
  # plot parameters
  analogData = AnalogData(100)
  analogPlot = AnalogPlot(analogData)

  print 'plotting data...'

  # open serial port
  ser = serial.Serial(strPort, 9600 )
  while True:
    try:
      line = ser.readline()
 #     print line
      data = [float(val) for val in line.split()]
      print data
      samples.append(data)
      if(len(data) == 5):
        analogData.add(data[0:4])
        analogPlot.update(analogData)

      norm_a = data[1]/data[0]
      norm_b = data[3]/data[2]
      if(norm_a < 0.1) and (norm_b) < 0.1:
        print 'off'
      else:
        invert_b = 1-norm_b
        position = (1-((norm_a + invert_b)/2.0))*100.0
        print "%.2f" % position
    except KeyboardInterrupt:
      print 'exiting'
      print samples
      break
  # close serial
  ser.flush()
  ser.close()

# call main
if __name__ == '__main__':
  main()
