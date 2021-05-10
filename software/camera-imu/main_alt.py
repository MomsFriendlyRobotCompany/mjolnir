#!/usr/bin/env python
# https://www.microchip.com/wwwproducts/en/ATSAMD21E18
# import attr
import time
from collections import deque
# import numpy as np
import sys
from slurm.rate import Rate
from math import pi
from colorama import Fore

import cv2
from opencv_camera import ThreadedCamera
from opencv_camera.color_space import ColorSpace
# from opencv_camera import Compressor

from tools.imu_driver_alt import IMUDriver
# from tools.imu_driver_alt import Parser
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


import numpy as np
import cv2


class Compressor:
    """
    Compressor allow you to serialize and compress an image using either JPEG
    or PNG compression.
    """

    _format = ".jpg"

    @property
    def format(self):
        return self._format

    @format.setter
    def format(self, fmt):
        """
        Set format to either .jpg jpg .png png
        """
        if fmt not in [".jpg", "jpg", ".png", "png"]:
            raise ValueError(f"Invalid format: {fmt}")
        if fmt.find(".") != 0:
            fmt = "." + fmt
        self._format = fmt

    def compress(self, img):
        jpeg_quality = 80
        params = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]
        # ok, cb = cv2.imencode(self._format, img, params)
        ok, cb = True, img
        if ok:
            cb = cb.tobytes()
        else:
            cb = None
        return cb

    def uncompress(self, img_bytes, shape):
        img = np.frombuffer(img_bytes, dtype=np.uint8)

        if len(shape) == 3:
            img = cv2.imdecode(img, cv2.IMREAD_COLOR)
        else:
            img = cv2.imdecode(img, cv2.IMREAD_GRAYSCALE)

        img = img.reshape(shape)
        return img







USECAM = 1

if USECAM:
    comp = Compressor()

    camera = ThreadedCamera()
    camera.open(1, fmt=ColorSpace.gray)
    camera.camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    camera.camera.set(cv2.CAP_PROP_EXPOSURE , 0.001)

    time.sleep(1)

port = "/dev/tty.usbmodem145401"
# port = "/dev/tty.usbmodem14501"
s = IMUDriver(port)

if len(sys.argv) > 1:
    filename = sys.argv[1]
else:
    filename = "data.pickle"

filename = f"save/{filename}"
print(f"Saving to {filename}")

rate = Rate(200)

try:
    start = time.monotonic()

    imuhz = Hertz(">> imu {:0.1f} hz",100)

    if USECAM:
        display = CameraDisplay()
        camhz = Hertz(">> camera {:0.1f} hz",100)

    while True:
        sdata = s.read()
        # print(sdata, flush=True)
        # time.sleep(1)
        if sdata is  None:
            print(f"{Fore.RED}*** oops: No IMU ***{Fore.RESET}")
            continue

        imuhz.increment()

        if USECAM:
            dt = time.monotonic() - start

            imgData = (None, None, None,)
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

                    imgData = (ff, f.shape, dt,)
                    # print(f">> image[{f.shape}]")
                    camhz.increment()
                else:
                    print(f"{Fore.RED}*** oops: No Image ***{Fore.RESET}")

            sdata += (imgData,)

        data.append(sdata)
        # rate.sleep()

except KeyboardInterrupt:
    print("ctrl-C")

finally:
    s.close()
    camera.close()

    if len(data) > 0:
        save.to_pickle(data, filename)

    print("\n\nbye ...\n")
