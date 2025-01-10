#!/usr/bin/env python3
import numpy as np
from picamera2 import Picamera2, Preview
import time


picam2 = Picamera2()
# picam2.start_preview(Preview.DRM)
c = picam2.create_video_configuration()
c["main"]["format"] = "BGR888"
picam2.configure(c)
picam2.start()

# set {format: "BGR888"}

time.sleep(1)

try:
    while True:
        np_array = picam2.capture_array()
        print(".")

except KeyboardInterrupt:
    print("\rCtrl-C")