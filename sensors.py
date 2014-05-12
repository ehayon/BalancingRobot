import time
import globals
from adxl345 import ADXL345

class Sensors:

  def init(self):
    print "Initializing ADXL345 Accelerometer"
    adxl345 = ADXL345()
    print "ADXL345 Initialized"
    readings = [0.0, 0.0, 0.0, 0.0, 0.0]
    i = 0
    while True:
      readings[i] = adxl345.read()[1]
      i = (i + 1) % 5
      time.sleep(0.0001)
      
      if i == 0: globals.ACCEL = (sum(readings) / float(len(readings)))
