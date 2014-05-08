from sensors import Sensors
import threading
import time
import globals

class BalancingRobot:

  def init(self):
    print "Initializing Balancing Robot Code"

if __name__ == "__main__":
  sensors = Sensors()
  st = threading.Thread(target=sensors.init)
  st.daemon = True
  st.start()

  while True:
    """
    Busy wait in the parent thread
    """
    print globals.ACCEL
    time.sleep(0.05)
