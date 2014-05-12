from sensors import Sensors
import threading
import time
import globals
from Adafruit_PWM_Servo_Driver import PWM
import RPi.GPIO as GPIO

class BalancingRobot:

  pwm = None

  calibrated_accel = 0.0

  INTEGRAL_SAMPLES = 20

  motor_output = 0.0

  Kp = 3.5
  Ki = 0.0
  Kd = 0.0

  def init(self):
    print "Initializing Balancing Robot Code"
  
    """ Calibrate the ADXL345 Accelerometer """

    self.pwm = PWM(0x40, debug=True)
    self.pwm.setPWMFreq(200)

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18, GPIO.OUT)
    GPIO.setup(23, GPIO.OUT)
    GPIO.setup(24, GPIO.OUT)
    GPIO.setup(25, GPIO.OUT)

    self.pwm.setPWM(0, 0, 0)
    self.pwm.setPWM(1, 0, 0)

    proceed = raw_input("Is the robot level? (yes|no):")
    if not (proceed == "yes" or proceed == "Yes"):
      return

    calibrated_sum = 0.0
    for i in range(0,100):
      print "Calibration step: %d (avg=%0.2f)" % (i, calibrated_sum / (i+1.0))
      calibrated_sum = calibrated_sum + globals.ACCEL
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
    integral_sum = 0.0
    prev_error = 0.0
    error = 0.0

    while True:
      # reset count if count > INTEGRAL_SAMPLES
      count = count % self.INTEGRAL_SAMPLES
      # reset integral sum if count resets
      if count == 0:
        integral_sum = 0.0

      # desired accelerometer balance position should be calibrated accel value
      accel_desired = self.calibrated_accel

      # keep track of previous error (used for derivative_error calculation)
      prev_error = error
      error = globals.ACCEL - accel_desired

      # sum up error over time
      integral_sum = integral_sum + error

      # compute derivative error
      derivative_error = prev_error - error

      pid = (self.Kp * error) + (self.Ki * (integral_sum / self.INTEGRAL_SAMPLES)) + (self.Kd * derivative_error)
      motor_output = pid

      # ensure that motor output doesnt exceed bounds [-100.0, 100.0]
      if motor_output < -100.0: motor_output = -100.0
      if motor_output > 100.0: motor_output = 100.0

      #print "PID corrective value = %0.2f" % (pid)
      #print "Output to motor = %0.2f" % (motor_output)


      # --- Output data to motor driver ---
      motor_dir = [False, True]

      if motor_output < 0: motor_dir = [not i for i in motor_dir] # flip all boolean values in list

      GPIO.output(18, motor_dir[0])
      GPIO.output(23, motor_dir[1])

      GPIO.output(25, motor_dir[0])
      GPIO.output(24, motor_dir[1])

      motor_output = abs(motor_output)

      self.pwm.setPWM(0, 0, int(4095*(motor_output/100.0)))
      self.pwm.setPWM(1, 0, int(4095*(motor_output/100.0)))
      #print globals.ACCEL
      count = count + 1
      time.sleep(0.0005)    


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
    wait for children threads to exit.
    """

    time.sleep(0.5)
