#!/usr/bin/env python
# https://www.microchip.com/wwwproducts/en/ATSAMD21E18
# import attr
import time

# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# import matplotlib.dates as mdates

from collections import deque
# import numpy as np
import sys
from slurm.rate import Rate
from math import pi
from colorama import Fore

import cv2
from opencv_camera import ThreadedCamera
from opencv_camera.color_space import ColorSpace
from opencv_camera import Compressor

from tools.imu_driver import IMUDriver
from tools.imu_driver import AGMPT, AGMPTBN055, AGMPTL
from tools.camera import CameraDisplay
from tools import save
from tools.hertz import Hertz

deg2rad = pi / 180.0
RAD2DEG = 180/pi
DEG2RAD = pi/180
FT2M = 0.3048   # feet to meters
MI2M = 1609.34  # miles to meters
# PACKET_LEN = 7
BUFFER_SIZE = 1000000

data = deque(maxlen=BUFFER_SIZE)
# cam_data = deque(maxlen=BUFFER_SIZE)

comp = Compressor()

camera = ThreadedCamera()
camera.open(1, fmt=ColorSpace.gray)
camera.camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
camera.camera.set(cv2.CAP_PROP_EXPOSURE , 0.001)

time.sleep(1)

port = "/dev/tty.usbmodem146401"
# port = "/dev/tty.usbmodem14501"
# parser = AGMPTBN055()
parser = AGMPTL()
s = IMUDriver(port, parser)

if len(sys.argv) > 1:
    filename = sys.argv[1]
else:
    filename = "data.pickle"
filename = f"save/{filename}"

rate = Rate(200)

try:
    start = time.monotonic()

    imuhz = Hertz(">> imu {:0.1f} hz",100)
    camhz = Hertz(">> camera {:0.1f} hz",100)

    display = CameraDisplay()

    while True:
        sdata = s.read()
        if sdata is  None:
            print(f"{Fore.RED}*** oops: No IMU ***{Fore.RESET}")
            continue

        imuhz.increment()
        dt = time.monotonic() - start

        imgData = (None, None,)
        if (imuhz.count % 5) == 0:
            ok,f = camera.read()
            if ok:
                f = cv2.flip(f, -1) # Flip camera vertically, bad mounting
                # print(f.shape)
                # display.imshow(f)

                ff = comp.compress(f)
                if ff is None:
                    print(f"{Fore.RED}*** oops: Image Compression Failed ***{Fore.RESET}")
                    continue

                imgData = (ff, f.shape,)
                # print(f">> image[{f.shape}]")
                camhz.increment()
            else:
                print(f"{Fore.RED}*** oops: No Image ***{Fore.RESET}")

        # sdata += (imgData, dt,)
        data.append(sdata)
        # rate.sleep()

except KeyboardInterrupt:
    print("ctrl-C")

finally:
    s.close()
    camera.close()

    # if len(data) > 0:
    #     save.to_pickle(filename)

    print("\n\nbye ...\n")
