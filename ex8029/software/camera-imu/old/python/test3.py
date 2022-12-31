#!/usr/bin/env python
# https://www.microchip.com/wwwproducts/en/ATSAMD21E18
import attr
import time
from serial import Serial
import struct
from math import log10, sin, cos, acos, atan2, asin, pi, sqrt
from collections import deque
from collections import namedtuple
import numpy as np
import cv2
from slurm.rate import Rate
import pickle
from opencv_camera import ThreadedCamera
from opencv_camera.color_space import ColorSpace

ImageIMU = namedtuple("ImageIMU","image accel gyro temperature timestamp")

deg2rad = pi / 180.0
RAD2DEG = 180/pi
DEG2RAD = pi/180
FT2M = 0.3048   # feet to meters
MI2M = 1609.34  # miles to meters
PACKET_LEN = 7

class AverageFilter(deque):
    def __init__(self, maxlen=5):
        super().__init__(maxlen=maxlen)
        for i in range(maxlen):
            # self.__setitem__(i, 0.0)
            self.append(np.zeros(3))

    def avg(self):
        avg = 0
        num = self.__len__()
        # print(num)
        for i in range(num):
            # print(self.__getitem__(i), end=" ")
            avg += self.__getitem__(i)
        return avg/num

def normalize3(x, y, z):
    """Return a unit vector"""
    norm = sqrt(x * x + y * y + z * z)

    # already a unit vector
    if norm == 1.0:
        return (x, y, z)

    inorm = 1.0/norm
    if inorm > 1e-6:
        x *= inorm
        y *= inorm
        z *= inorm
    else:
        raise ZeroDivisionError(f'norm({x:.4f}, {y:.4f}, {z:.4f},) = {inorm:.6f}')
    return (x, y, z,)

# @attr.s(slots=True)
class IMUDriver:
    __slots__ = ["s"]
    # s = attr.ib(default=None)
    # port = attr.ib()
    # speed
    def __init__(self, port):
        speed = 115200
        self.s = Serial(port, speed, timeout=0.01)

    def close(self):
        self.s.close()

    def read(self):
        data_size = PACKET_LEN*4
        self.s.reset_input_buffer()
        self.s.write(b"g")
        bad = True
        for _ in range(data_size):
            m = self.s.read(1)
            if m == b"":
                # print(".", end="", flush=True)
                continue
            if m != b"\xff":
                # print("x", end="", flush=True)
                continue
            else:
                bad = False
                break
        if bad:
            return None
        d = self.s.read(data_size)
        num = len(d)
        while num != data_size:
            d += self.s.read(num-len(d))

        # msg = struct.unpack("fffffffffff", d)
        # a = msg[:3]   # g's
        # g = msg[3:6]  # rads/sec
        # m = msg[6:9]  # normalized to uTesla
        # p = msg[9]    # pressure hPa
        # p = self.height(p)
        # t = msg[10]   # C
        # t = t*9/5+32  # F
        msg = struct.unpack("fffffff", d)
        a = msg[:3]   # g's
        g = msg[3:6]  # rads/sec
        t = msg[6]    # C

        return a,g,t

    def compensate(self, accel, mag=None):
        """
        """
        try:
            ax, ay, az = normalize3(*accel)

            pitch = asin(-ax)

            if abs(pitch) >= pi/2:
                roll = 0.0
            else:
                roll = asin(ay/cos(pitch))

            if mag:
                # mx, my, mz = mag
                mx, my, mz = normalize3(*mag)
                x = mx*cos(pitch)+mz*sin(pitch)
                y = mx*sin(roll)*sin(pitch)+my*cos(roll)-mz*sin(roll)*cos(pitch)
                heading = atan2(y, x)

                # wrap heading between 0 and 360 degrees
                if heading > 2*pi:
                    heading -= 2*pi
                elif heading < 0:
                    heading += 2*pi
            else:
                heading = None

            # if self.angle_units == Angle.degrees:
            # roll    *= RAD2DEG
            # pitch   *= RAD2DEG
            # heading *= RAD2DEG
            # elif self.angle_units == Angle.quaternion:
            #     return Quaternion.from_euler(roll, pitch, heading)

            return (roll, pitch, heading,)

        except ZeroDivisionError as e:
            print('Error', e)
            # if self.angle_units == Angle.quaternion:
            #     return Quaternion(1, 0, 0, 0)
            # else:
            return (0.0, 0.0, 0.0,)

    def height(self, p):
        """
        given pressure in hPa, returns altitude in meters.
        """
        h = (1 - pow(p / 1013.25, 0.190263)) * 44330.8
        return h


def savePickle(data, filename):
    with open(filename, 'wb') as fd:
            d = pickle.dumps(data)
            fd.write(d)


af = AverageFilter(5)
gf = AverageFilter(5)
#
# for i in range(20):
#     v = np.array([i,i,i])
#     a.append(0.1*v)
#     print(a.avg())
#
# exit(0)

loop_rate = Rate(100)
images = []
last = time.monotonic()
loop = 1
rate = 0.0

# port = "/dev/tty.usbmodem14401"
port = "/dev/tty.usbmodem14501"
s = IMUDriver(port)

# aa = AverageFilter(10)
path = 0
camera = ThreadedCamera()
res = None
# res = (480,640)
# res = (720,2560)
camera.open(path, resolution=res, fmt=ColorSpace.gray)

aa = af.avg()
gg = gf.avg()
t = 0

try:
    start = time.monotonic()
    while True:
        if loop % 5:
            aa = af.avg()
            gg = gf.avg()
            ok, img = camera.read()

            if ok:
                h,w = img.shape
                si = cv2.resize(img, (w//2,h//2))
                cv2.imshow('capture', si)
                ch = cv2.waitKey(20)
                if ch == ord('q'):
                    break
                elif ch == ord('s'):
                    imgimu = ImageIMU(img,aa,gg,t,(time.monotonic() - start))
                    images.append(imgimu)

            else:
                img = np.array((1,1))

            # roll, pitch, _ = s.compensate(aa)
            # roll, pitch, _ = 0,0,0
            # roll  *= RAD2DEG
            # pitch *= RAD2DEG
            # yaw   *= RAD2DEG

            print(f"R: {rate:3.0f} I: {img.shape} A: {aa[0]:5.2f} {aa[1]:5.2f} {aa[2]:5.2f} G: {gg[0]:5.2f} {gg[1]:5.2f} {gg[2]:5.2f} T: {t:2.1f}", end="\r")
            # if ok and 100%20 == 0:
            #     print(ok, img.shape)

        ret = s.read()
        # ret = None
        if ret:
            a,g,t = ret
            # a = (a[0]-0.11, a[1]-0.82, a[2])
            af.append(np.array(a))
            gf.append(np.array(g))
            #
            # roll, pitch, _ = s.compensate(a)
            # roll  *= RAD2DEG
            # pitch *= RAD2DEG
            # yaw   *= RAD2DEG

            # if loop % 20 == 0:
            #     # print(f"R: {rate:6.1f} A: {a[0]:5.3f} {a[1]:5.3f} {a[2]:5.3f} G: {g[0]:5.2f} {g[1]:5.2f} {g[2]:5.2f} M: {m[0]:5.1f} {m[1]:5.1f} {m[2]:5.1f}  H: {p:5.1f} T: {t:3.1f}", end="\r")
            #     print(f"R: {rate:6.1f} A: {a[0]:5.3f} {a[1]:5.3f} {a[2]:5.3f} G: {g[0]:5.2f} {g[1]:5.2f} {g[2]:5.2f} T: {t:3.1f}", end="\r")
            #     # print(f"roll: {roll:6.1f} pitch: {pitch:6.1f} yaw: {yaw:6.1f}", end="\r")

        if loop % 100 == 0:
            now = time.monotonic()
            rate = 100/(now - last)
            last = now
            # print(f">> Rate: {rate:0.3f} Hz")

        loop += 1

        loop_rate.sleep()

except KeyboardInterrupt:
    # s.close()
    print("ctrl-C")
finally:
    camera.close()
    cv2.destroyAllWindows()
    if len(images) > 0:
        savePickle(images, "images.pickle")
    print("\n\nbye ...\n")
