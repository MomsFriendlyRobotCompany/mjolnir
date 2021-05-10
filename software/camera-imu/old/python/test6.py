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

# from opencv_camera import ThreadedCamera
# from opencv_camera.color_space import ColorSpace

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


def savePickle(data, filename):
    with open(filename, 'wb') as fd:
            d = pickle.dumps(data)
            fd.write(d)

port = "/dev/tty.usbmodem14401"
# port = "/dev/tty.usbmodem14501"
parser = AGMPTBN055()
s = IMUDriver(port, parser)

if len(sys.argv) > 1:
    filename = sys.argv[1]
else:
    filename = "data.pickle"

rate = Rate(200)


fig, ax = plt.subplots(1, 1)
ax.set_aspect(1)

mag_x = deque(maxlen=BUFFER_SIZE)
mag_y = deque(maxlen=BUFFER_SIZE)
mag_z = deque(maxlen=BUFFER_SIZE)

def plot(ax, x,y,z,title):
    # Clear all axis
    ax.cla()

    # Display the sub-plots
    ax.scatter(x, y, color='r', label="X-Y")
    ax.scatter(y, z, color='g', label="Y-Z")
    ax.scatter(z, x, color='b', label="Z-X")

    ax.grid()
    ax.legend()
    ax.set_title(title)

    # Pause the plot for INTERVAL seconds
    plt.pause(0.1)

try:
    start = time.monotonic()
    start_hz = start
    cnt = 0

    while True:
        agmpt = s.read()
        if agmpt is  None:
            print(f"{Fore.RED}*** oops: {agmpt} ***{Fore.RESET}")
            continue

        if cnt == 100:
            # try:
            # a = agmpt[2]
            # a = len(agmpt)
            # print(f">> {a}")
            now = time.monotonic()
            hz = cnt/(now - start_hz)
            cnt = 0
            start_hz = now
            title = f"{hz:0.1f} Hz"

            if 0:
                plot(ax,mag_x,mag_y,mag_z, title)
            else:
                print(">>", title)

            # except Exception as e:
            #     print(f"{Fore.RED}*** {e} ***{Fore.RESET}")
            #     continue
        dt = time.monotonic() - start

        # save data for real-time plotting
        mag_x.append(agmpt[2][0])
        mag_y.append(agmpt[2][1])
        mag_z.append(agmpt[2][2])

        # append timestamp
        agmpt += (dt,)
        data.append(agmpt)

        cnt += 1
        rate.sleep()

except KeyboardInterrupt:
    print("ctrl-C")
finally:
    s.close()
    # camera.close()
    # cv2.destroyAllWindows()
    if len(data) > 0:
        print(f">> Collected {len(data)} data points, saving to {filename}")
        savePickle(data, filename)
    print("\n\nbye ...\n")
