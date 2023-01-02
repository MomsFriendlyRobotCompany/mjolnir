#!/usr/bin/env python3

import cv2
import pickle
import numpy as np
import time
from tools.imu_driver_alt import AccelGyroMag, TempPress, Light
from tools.camera import CameraDisplay, DisparityDisplay
from matplotlib import pyplot as plt

from tools import save



# display = CameraDisplay("video")
# display = DisparityDisplay("video")

filename = "save/data-save.pickle"
data = save.from_pickle(filename)
# print(data)

# FIXME: [AGM, TP, ts, (img, size, ts), ts]
for d in data[:100]:
    cd = d[3]
    img, sz, ts = cd
    if img:
        img = np.frombuffer(img, dtype=np.uint8).reshape(*sz)
        # quit, key = display.imshow(img)
        # if quit:
        #     break
        # print(f">> {img.shape} {ts}", end="\r")
        # time.sleep(0.05)
