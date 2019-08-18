import sys

# Takes the V-REP Remote API
sys.path.append('../../V-REP/components/hexapod/hexapod_vrep/vrep_api')
sys.path.append('../project_1')
import vrep
import numpy as np
import time
import cv2
from ePuckVRep import ePuck


class Controller():
    def __init__(self, host='127.0.0.1', port=19997, debug=True):
        self.rb = ePuck(host, port, debug)
        self.rb.connect()
        if self.rb.is_connected():
            print("CONNECTED TO API")

            # Start simulation
            self.rb.startsim()

            # Enable all components
            for i in ("floor", "proximity", "motor_position"):
                self.rb.enable(i)
        else:
            raise Exception("[ERROR] CANNOT CONNECT TO API")

    def show_image(self, image):
        pass

    def compute(self):
        self.val = 1 if self.val == 0 else 0
        self.rb.set_front_led(self.val)
        self.rb.set_body_led(self.val)

    def init_work(self):
        self.rb.set_motors_speed(0, 0)
        self.rb.set_front_led(0)
        self.rb.set_body_led(0)
        self.val = 0
        
    def start(self):
        self.init_work()
        while True:
            self.compute()
            self.rb.step()


if __name__ == "__main__":
    print "STARTING EPUCK1"
    Controller(port=19997, host='158.49.227.95').start()