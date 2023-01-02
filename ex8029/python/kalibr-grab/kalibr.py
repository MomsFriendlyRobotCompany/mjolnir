import numpy as np
import cv2
import time # monotonic_ns for timestamp
from datetime import datetime as dtime # naming directory
from pathlib import Path
from collections import deque
# from opencv_camera import bgr2gray
import csv

class KalibrWriteData:
    camera = deque()
    imu = deque()

    def pushImage(self, left, right):
        self.camera.append((left,right,time.monotonic_ns()))

    def pushImu(self, wx,wy,wz,ax,ay,az):
        d = (
            time.monotonic_ns(), # ns
            wx, wy, wz,          # rad/sec
            ax, ay, az           # m/sec^2
        )
        self.imu.append(d)

    def write(self, path):
        print(f"{self.__class__.__name__} image: {len(self.camera)} imu: {len(self.imu)}")
        if len(self.camera) > 0:
            self._writeCamera(path)
            print(f"{self.__class__.__name__} saved images")
        if len(self.imu) > 0:
            self._writeImu(path)
            print(f"{self.__class__.__name__} saved imu")

    def _writeCamera(self, path):
        """
        Save as a kalibr dataset
        """
        basepath = Path(path)
        left = basepath.joinpath("cam0")
        left.mkdir(parents=True,exist_ok=True)
        right = basepath.joinpath("cam1")
        right.mkdir(parents=True,exist_ok=True)

        for img_l, img_r, ts in self.camera:
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

    def clear(self):
        self.camera.clear()
        self.imu.clear()

    # def image(self):
    #     for im in self.camera:
    #         yield im

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