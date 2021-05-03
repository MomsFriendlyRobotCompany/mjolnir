#!/usr/bin/env python
# https://www.microchip.com/wwwproducts/en/ATSAMD21E18
# import attr
import time

# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# import matplotlib.dates as mdates

from collections import deque
import numpy as np
import sys
from math import pi
# import pickle
from colorama import Fore

from slurm.rate import Rate

from tools.imu_driver import IMUDriver
from tools.imu_driver import AGMPT, AGMPTBN055, AGMPTL
from tools import save

deg2rad = pi / 180.0
RAD2DEG = 180/pi
DEG2RAD = pi/180
FT2M = 0.3048   # feet to meters
MI2M = 1609.34  # miles to meters
PACKET_LEN = 7
BUFFER_SIZE = 1000000

data = deque(maxlen=BUFFER_SIZE)

# class MagCal:
#     def __init__(self, BUFFER_SIZE=1000000):
#         self.fig, self.ax = plt.subplots(1, 1)
#         self.ax.set_aspect(1)
#
#         self.mag_x = deque(maxlen=BUFFER_SIZE)
#         self.mag_y = deque(maxlen=BUFFER_SIZE)
#         self.mag_z = deque(maxlen=BUFFER_SIZE)
#         self.ts = deque(maxlen=BUFFER_SIZE)
#
#     def push(self, x,y,z, ts=None):
#         # save data for real-time plotting
#         self.mag_x.append(x)
#         self.mag_y.append(y)
#         self.mag_z.append(z)
#         if ts:
#             self.ts.append(ts)
#
#     def save(self, filename=None):
#         if len(self.ts) > 0:
#             data = (self.mag_x,self.mag_y,self.mag_z,self.ts,)
#         else:
#             data = (self.mag_x,self.mag_y,self.mag_z,)
#
#         if filename is None:
#             filename = "mag-cal-data.pickle"
#
#         with open(filename, 'wb') as fd:
#             d = pickle.dumps(data)
#             fd.write(d)
#
#     def plot(self, title=None):
#         # Clear all axis
#         self.ax.cla()
#         x = self.mag_x
#         y = self.mag_y
#         z = self.mag_z
#
#         # Display the sub-plots
#         self.ax.scatter(x, y, color='r', label="X-Y")
#         self.ax.scatter(y, z, color='g', label="Y-Z")
#         self.ax.scatter(z, x, color='b', label="Z-X")
#         self.ax.grid()
#         self.ax.legend()
#
#         if title is None:
#             title = "MagCal"
#         self.ax.set_title(title)
#
#         # Pause the plot for INTERVAL seconds
#         plt.pause(0.1)

# def savePickle(data, filename):
#     with open(filename, 'wb') as fd:
#         d = pickle.dumps(data)
#         fd.write(d)

def main(filename, port):

    parser = AGMPTL()
    s = IMUDriver(port, parser)

    rate = Rate(200)

    cal = MagCal()

    try:
        start = time.monotonic()
        start_hz = start
        cnt = 0

        while True:
            agmpt = s.read()
            if agmpt is  None:
                print(f"{Fore.RED}*** oops: {agmpt} ***{Fore.RESET}")
                continue

            if cnt == 10:
                now = time.monotonic()
                hz = cnt/(now - start_hz)
                cnt = 0
                start_hz = now
                title = f"{hz:0.1f} Hz"

                if 0:
                    cal.plot(title)
                else:
                    print(">>", title)

            dt = time.monotonic() - start
            cal.push(agmpt[2][0],agmpt[2][1],agmpt[2][2],dt)

            # append timestamp
            agmpt += (dt,)
            data.append(agmpt)

            cnt += 1
            rate.sleep()

    except KeyboardInterrupt:
        print("ctrl-C")
    finally:
        s.close()
        if len(data) > 0:
            print(f">> Collected {len(data)} data points, saving to {filename}")
            # save.to_pickle(data, filename)
        print("\n\nbye ...\n")

if __name__ == "__main__":
    port = "/dev/tty.usbmodem14301"
    # port = "/dev/tty.usbmodem14501"

    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = "data.pickle"
    filename = f"save/{filename}"

    main(filename, port)
