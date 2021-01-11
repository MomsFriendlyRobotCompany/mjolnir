#!/usr/bin/env python
# https://www.microchip.com/wwwproducts/en/ATSAMD21E18
# import attr
import time
from collections import deque
import numpy as np
# import cv2
from slurm.rate import Rate
from math import pi
import pickle

# from opencv_camera import ThreadedCamera
# from opencv_camera.color_space import ColorSpace

from imu_driver import IMUDriver
from imu_driver import AGMPT, agmpt_t

# ImageIMU = namedtuple("ImageIMU","image accel gyro temperature timestamp")

deg2rad = pi / 180.0
RAD2DEG = 180/pi
DEG2RAD = pi/180
FT2M = 0.3048   # feet to meters
MI2M = 1609.34  # miles to meters
PACKET_LEN = 7

data = deque(maxlen=10000)


def savePickle(data, filename):
    with open(filename, 'wb') as fd:
            d = pickle.dumps(data)
            fd.write(d)

port = "/dev/tty.usbmodem14601"
# port = "/dev/tty.usbmodem14501"
s = IMUDriver(port, AGMPT())

rate = Rate(200)

try:
    start = time.monotonic()
    cnt = 1

    while True:
        agmpt = s.read()
        if agmpt is  None:
            print(f"oops: {agmpt}")
        else:
            if (cnt % 200) == 0:
                a,g,m,p,t = agmpt
                print(f">> {a}")
            dt = time.monotonic() - start
            # m = agmpt_t(a,g,m,p,t, dt)
            # print(f">> {m}")
            agmpt += (dt,)
            data.append(agmpt)

        # time.sleep(0.001)
        cnt += 1
        rate.sleep()

except KeyboardInterrupt:
    print("ctrl-C")
finally:
    s.close()
    # camera.close()
    # cv2.destroyAllWindows()
    if len(data) > 0:
        savePickle(data, "data.pickle")
    print("\n\nbye ...\n")
