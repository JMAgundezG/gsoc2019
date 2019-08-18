import sys
import time
from EV3_LEGO_controller import EV3Controller
import cv2
import numpy as np



if __name__ == "__main__":
    suffix = ""
    if len(sys.argv) > 1:
        if int(sys.argv[1]) > 0:
            suffix += "#" + str(int(sys.argv[1]) - 1)
    ip = "158.49.227.10"
    ip = "127.0.0.1"
    l = EV3Controller(ip, 19998, suffix)
    l.set_debug(True)
    l.set_speed_left(10)
    l.set_speed_right(10)
    j = 0
    print suffix
    raw_input()