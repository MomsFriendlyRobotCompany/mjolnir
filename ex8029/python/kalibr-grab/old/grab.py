#!/usr/bin/env python3

'''
--------------------------------------------------
Image: 720x1280
Markers.checkerboard: (7, 10)
Intrinsic Camera Parameters
--------------------------------------------------
 [Camera 1]
  f(x,y): 1129.4 1127.4 px
  principlePoint(x,y): 640.8 357.0 px
  distCoeffs [ 1.39107250e-01 -5.25456587e-01 -1.30556613e-04  9.07833455e-04
  9.33127229e-01]
 [Camera 2]
  f(x,y): 1128.5 1127.2 px
  principlePoint(x,y): 652.1 354.6 px
  distCoeffs [ 1.28455592e-01 -3.69658433e-01  5.09480888e-04 -2.52525323e-04
  4.11286905e-01]
--------------------------------------------------
Extrinsic Camera Parameters
--------------------------------------------------
  R [[ 0.99999112 -0.00148358 -0.00394393]
 [ 0.0014944   0.99999512  0.00274168]
 [ 0.00393984 -0.00274755  0.99998846]]
  T[meter] [[-0.03134446]
 [-0.00025078]
 [-0.00159318]]
  E [[ 1.39283028e-06  1.59385949e-03 -2.46405370e-04]
 [-1.46967190e-03 -8.37569159e-05  3.13503789e-02]
 [ 2.03932801e-04 -3.13446763e-02 -8.69255963e-05]]
  F [[ 4.60170665e-08  5.27552785e-05 -2.80562760e-02]
 [-4.86110558e-05 -2.77543141e-06  1.20330732e+00]
 [ 2.48110107e-02 -1.20419456e+00  1.00000000e+00]]
'''
from serial import Serial
import numpy as np
import cv2
import time # monotonic_ns for timestamp
from datetime import datetime as dtime # naming directory
from pathlib import Path
from collections import deque
from opencv_camera import bgr2gray
import csv
from yivo import Yivo
from yivo.packet import ImuAGT
from serial import Serial

cap = cv2.VideoCapture(0) #, cv2.CAP_AVFOUNDATION)
# ok, frame = cap.read()

width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
fps = int(cap.get(5))
print(f">> Camera: {width}x{height} @ {fps}")

# why don't these work?!
# cap.set(cv2.CAP_PROP_FPS, 210)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
# cap.set(cv2.CAP_PROP_FPS, 210)


class KalibrWriteData:
    camera = deque()
    imu = deque()

    def pushImage(self, img):
        self.camera.append((img,time.monotonic_ns()))

    def pushImu(self, imu):
        d = (
            time.monotonic_ns(),    # ns
            imu.wx, imu.wy, imu.wz, # rad/sec
            imu.ax, imu.ay, imu.az  # m/sec^2
        )
        self.imu.append(d) # timestamp in imu message

    def write(self, path):
        if len(self.camera) > 0:
            self._writeCamera(path)
        if len(self.imu) > 0:
            self._writeImu(path)

    def _writeCamera(self, path):
        """
        Save as a kalibr dataset
        """
        basepath = Path(path)
        left = basepath.joinpath("cam0")
        left.mkdir(parents=True,exist_ok=True)
        right = basepath.joinpath("cam1")
        right.mkdir(parents=True,exist_ok=True)

        # epoch = self.camera[0][1]
        r,c = self.camera[0][0].shape[:2]
        for im, ts in self.camera:
            edge = c//2
            img_l,img_r = im[:,:edge], im[:,edge:]

            # no compress does appear to make diff, 2x bigger file
            # compress = [cv2.IMWRITE_PNG_COMPRESSION, 0]
            compress = None

            lp = left.joinpath(str(ts) + ".png")
            cv2.imwrite(str(lp), img_l, compress)

            rp = right.joinpath(str(ts) + ".png")
            cv2.imwrite(str(rp), img_r, compress)

    def _writeImu(self, path):
        basepath = Path(path)
        # this should exist, but incase not, create it
        basepath.mkdir(parents=True,exist_ok=True)

        p = basepath.joinpath("imu0.csv")
        with p.open('w', newline='') as fd:
            writer = csv.writer(fd)
            writer.writerows(self.imu)

    def image(self):
        for im in self.camera:
            yield im

    # def imu(self):
    #     for msg in self.imu:
    #         yield msg

    # @property
    # def p_left(self):
    #     return self.pl

    # @p_left.setter
    # def p_left(self, p):
    #     if not isinstance(p,np.ndarray) or p.shape != (3,4):
    #         raise ValueError(f"P must be numpy array shape (3,4), not {type(p)} {p.shape}")
    #     self.pl = p

    # @property
    # def p_right(self):
    #     return self.pr

    # @p_right.setter
    # def p_right(self, p):
    #     if not isinstance(p,np.ndarray) or p.shape != (3,4):
    #         raise ValueError(f"P must be numpy array shape (3,4), not {type(p)} {p.shape}")
    #     self.pr = p
    # def setLeftCamera(self, k, p, d):
    #     self.Kl = k
    #     self.Pl = p
    #     self.dl = d

    # def setRightCamera(self, k, p, d):
    #     self.Kr = k
    #     self.Pr = p
    #     self.dr = d



ok = False
while (ok is False):
    time.sleep(1)
    ok, frame = cap.read()
    print('.', end="", flush=True)

sz = frame.shape
print(f">> Image: {sz[0]}x{sz[1]}")

s = Serial()
s.port = "/dev/tty.usbmodem14601"
s.baud = 1000000
s.timeout = 1
s.open()

yivo = Yivo()


record = False

sd = KalibrWriteData()

count = 0

try:
    while True:
        if s.is_open:
            s.write(b'g') # start data stream

        ok, frame = cap.read()

        if not ok:
            print("** no image **")
            break

        frame = bgr2gray(frame)
        cv2.imshow("win", frame)

        if record:
            sd.pushImage(frame)

            msg = yivo.read_packet(s)
            if msg:
                if type(msg) == ImuAGT:
                    sd.pushImu(msg)

        c = cv2.waitKey(10)
        if c == 27:
            break
        elif c == ord('c'):
            sd.pushImage(frame)
        elif c == ord('s'):
            cv2.imwrite("./save/" + str(count) + ".png", frame)
            count += 1
        elif c == ord('r'):
            record = not record
            print(f">> Recording data: {record}")

except KeyboardInterrupt:
    print("\nctl-c ...")

finally:
    sd.write(f"./data/{dtime.now().strftime('%m.%d.%Y-%H-%M-%S')}")
    cap.release()
    cv2.destroyAllWindows()
