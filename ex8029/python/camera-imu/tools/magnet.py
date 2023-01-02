import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.dates as mdates

from collections import deque


class MagCal:
    def __init__(self, BUFFER_SIZE=1000000):
        self.fig, self.ax = plt.subplots(1, 1)
        self.ax.set_aspect(1)

        self.mag_x = deque(maxlen=BUFFER_SIZE)
        self.mag_y = deque(maxlen=BUFFER_SIZE)
        self.mag_z = deque(maxlen=BUFFER_SIZE)
        self.ts = deque(maxlen=BUFFER_SIZE)

    def push(self, x,y,z, ts=None):
        # save data for real-time plotting
        self.mag_x.append(x)
        self.mag_y.append(y)
        self.mag_z.append(z)
        if ts:
            self.ts.append(ts)

    def save(self, filename=None):
        if len(self.ts) > 0:
            data = (self.mag_x,self.mag_y,self.mag_z,self.ts,)
        else:
            data = (self.mag_x,self.mag_y,self.mag_z,)

        if filename is None:
            filename = "mag-cal-data.pickle"

        with open(filename, 'wb') as fd:
            d = pickle.dumps(data)
            fd.write(d)

    def plot(self, title=None):
        # Clear all axis
        self.ax.cla()
        x = self.mag_x
        y = self.mag_y
        z = self.mag_z

        # Display the sub-plots
        self.ax.scatter(x, y, color='r', label="X-Y")
        self.ax.scatter(y, z, color='g', label="Y-Z")
        self.ax.scatter(z, x, color='b', label="Z-X")
        self.ax.grid()
        self.ax.legend()

        if title is None:
            title = "MagCal"
        self.ax.set_title(title)

        # Pause the plot for INTERVAL seconds
        plt.pause(0.1)
