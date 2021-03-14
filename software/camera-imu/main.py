#!/usr/bin/env python
# https://www.microchip.com/wwwproducts/en/ATSAMD21E18
# import attr
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.dates as mdates

from collections import deque
import numpy as np
import sys
from slurm.rate import Rate
from math import pi
import pickle
from colorama import Fore

import cv2
from opencv_camera import ThreadedCamera
from opencv_camera.color_space import ColorSpace
from opencv_camera import Compressor

from imu_driver import IMUDriver
from imu_driver import AGMPT, AGMPTBN055

# ImageIMU = namedtuple("ImageIMU","image accel gyro temperature timestamp")

deg2rad = pi / 180.0
RAD2DEG = 180/pi
DEG2RAD = pi/180
FT2M = 0.3048   # feet to meters
MI2M = 1609.34  # miles to meters
PACKET_LEN = 7
BUFFER_SIZE = 1000000

data = deque(maxlen=BUFFER_SIZE)
# cam_data = deque(maxlen=BUFFER_SIZE)

comp = Compressor()

camera = ThreadedCamera()
camera.open(0, fmt=ColorSpace.gray)
camera.camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
camera.camera.set(cv2.CAP_PROP_EXPOSURE , 0.001)

time.sleep(1)
#
# exit(0)


def savePickle(data, filename):
    with open(filename, 'wb') as fd:
            d = pickle.dumps(data)
            fd.write(d)

port = "/dev/tty.usbmodem145301"
# port = "/dev/tty.usbmodem14501"
parser = AGMPTBN055()
s = IMUDriver(port, parser)

if len(sys.argv) > 1:
    filename = sys.argv[1]
else:
    filename = "data.pickle"

rate = Rate(200)

class Hertz:
    def __init__(self, format, rollover):
        self.rollover = rollover
        self.format = format
        self.reset()

    def increment(self):
        self.count += 1

        if (self.count % self.rollover) == 0:
            now = time.monotonic()
            hz = self.count/(now - self.start_hz)
            self.count = 0
            self.start_hz = now
            print(self.format.format(hz))

    def reset(self):
        self.start_hz = time.monotonic()
        self.count = 0

try:
    start = time.monotonic()

    imuhz = Hertz(">> imu {:0.1f} hz",100)
    camhz = Hertz(">> camera {:0.1f} hz",20)

    while True:
        agmpt = s.read()
        if agmpt is  None:
            print(f"{Fore.RED}*** oops: No IMU ***{Fore.RESET}")
            continue

        imuhz.increment()
        dt = time.monotonic() - start

        if (imuhz.count % 5) == 0:
            ok,f = camera.read()
            if ok:
                f = cv2.flip(f, -1) # Flip camera vertically, bad mounting
                # print(f.shape)
                ff = comp.compress(f)
                if ff is None:
                    print(f"{Fore.RED}*** oops: No Camera ***{Fore.RESET}")
                    continue

                agmpt += ((ff, f.shape), dt,)
                # print(f">> image[{f.shape}]: {len(ff)}")
                # camhz.increment()
        else:
            # append timestamp
            agmpt += (dt,)
        data.append(agmpt)
        rate.sleep()

except KeyboardInterrupt:
    print("ctrl-C")
finally:
    s.close()
    camera.close()
    # cv2.destroyAllWindows()

    if len(data) > 0:
        print(f">> Collected {len(data)} data points, saving to {filename}")
        savePickle(data, filename)
    #
    # if len(cam_data) > 0:
    #     savePickle(cam_data, "cam.pickle")

    print("\n\nbye ...\n")
