from sensors import Sensors
import threading
import time
import globals



class BalancingRobot:

  calibrated_accel = 0.0

  INTEGRAL_SAMPLES = 100

  motor_output = 0.0

  Kp = 0.2
  Ki = 0.05
  Kd = 0.09

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
    self.calibrated_accel = calibrated_sum / 100.0

    # calibration complete
    print "Calibrated accelerometer value is %0.2f" % (self.calibrated_accel)



  def run(self):
    """ Balancing Robot Thread """
    self.init()

    print "Attemptimg to balance the robot"

    print "PID gains: Kp = %0.2f\tKi = %0.2f\tKd=%0.2f" % (self.Kp, self.Ki, self.Kd)

    count = 0
    integral_sum = 0
    prev_error = 0.0
    error = 0.0

    while True:
      count = count % self.INTEGRAL_SAMPLES
      if count == 0:
        integral_sum = 0

      accel_desired = self.calibrated_accel

      prev_error = error
      error = globals.ACCEL[1] - accel_desired

      integral_sum = integral_sum + error
      derivative_error = prev_error - error

      pid = (self.Kp * error) + (self.Ki * (integral_sum / self.INTEGRAL_SAMPLES)) + (self.Kd * derivative_error)

      motor_output = pid
     
      if motor_output < -100.0: motor_output = -100.0
      if motor_output > 100.0: motor_output = 100.0

      #print "PID corrective value = %0.2f" % (pid)
      print "Output to motor = %0.2f" % (motor_output)
      
      count = count + 1
      time.sleep(0.05)    


if __name__ == "__main__":
  # initialize the sensor reading thread
  sensors = Sensors()
  st = threading.Thread(target=sensors.init)
  st.daemon = True
  st.start()

  # initialize the balancing robot thread
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
