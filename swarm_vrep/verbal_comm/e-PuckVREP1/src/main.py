import sys
import time
from epuck_controller import EPuckController
import cv2
import numpy as np



if __name__ == "__main__":
    suffix = ""
    if len(sys.argv) > 1:
        if int(sys.argv[1]) > 0:
            suffix += "#" + str(int(sys.argv[1]) - 1)
    ip = "158.49.227.10"
    ip = "127.0.0.1"
    l = EPuckController(ip, 19998, suffix)
    l.set_debug(True)
    l.set_speed_left(100)
    l.set_speed_right(-100)
    j = 0
    print suffix
    raw_input()