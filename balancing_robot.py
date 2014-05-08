from sensors import Sensors
import threading
import time
import globals



class BalancingRobot:

  calibrated_accel = 0.0

  def init(self):
    print "Initializing Balancing Robot Code"
  
    """ Calibrate the ADXL345 Accelerometer """

    proceed = raw_input("Is the robot level? (yes|no):")
    if not (proceed == "yes" or proceed == "Yes"):
      return

    calibrated_sum = 0.0
    for i in range(0,100):
      print "Calibration step: %d (avg=%0.2f)" % (i, calibrated_sum / (i+1.0))
      calibrated_sum = calibrated_sum + globals.ACCEL[1]
      time.sleep(0.05)
    calibrated_accel = calibrated_sum / 100.0

    # calibration complete
    print "Calibrated accelerometer value is %0.2f" % (calibrated_accel)



  def run(self):
    """ Balancing Robot Thread """
    self.init()

    print "Attemptimg to balance the robot"

    time.sleep(0.05)


if __name__ == "__main__":
  sensors = Sensors()
  st = threading.Thread(target=sensors.init)
  st.daemon = True
  st.start()

  balancing_robot = BalancingRobot()
  bt = threading.Thread(target=balancing_robot.run)
  bt.daemon = True
  bt.start()

  while True:
    """
    Busy wait in the parent thread
    """
    #print globals.ACCEL
    time.sleep(0.2)
