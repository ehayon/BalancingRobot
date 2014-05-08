import time
import globals
from adxl345 import ADXL345

class Sensors:

  def init(self):
    print "Initializing ADXL345 Accelerometer"
    adxl345 = ADXL345()
    print "ADXL345 Initialized"
    while True:
      globals.ACCEL = adxl345.read()

      time.sleep(0.01)

