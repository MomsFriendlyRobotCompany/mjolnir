#!/usr/bin/env python3
from pyray import *
from enum import IntEnum
import cv2
import numpy as np
import time
from picamera2 import Picamera2
from pprint import pprint

# You can OR these together
# LOG_ALL: Show all log messages
# LOG_TRACE: Show trace messages
# LOG_DEBUG: Show debug messages
# LOG_INFO: Show info messages
# LOG_WARNING: Show warning messages
# LOG_ERROR: Show error messages
# LOG_FATAL: Show fatal error messages
# LOG_NONE: Don't show any log messages

set_trace_log_level(LOG_ERROR)

# Initialize the window
init_window(800, 480, "My 2D Window")

# Set the target frames per second
set_target_fps(60)

picam2 = Picamera2()
# picam2.start_preview(Preview.DRM)

new_w = 640
new_h = 480
# new_w = 720
# new_h = 640
new_w = 1920
new_h = 1080
c = picam2.create_video_configuration()
# c["main"]["format"] = "BGR888"
# c["main"]["size"] = (new_w, new_h)
# c["main"]["stride"] = new_w
# c["main"]["framesize"] = new_w * new_h * 3

cfg = {
    "format": "YUV420",
    # "format": "BGR888",
    "size": (new_w, new_h),
    # "stride": new_w * 3,
    # "framesize": new_w * new_h * 3,
}

c["main"] = cfg

picam2.configure(c)
picam2.start()

pprint(c)

time.sleep(1)

# Game loop
while not window_should_close():
    # ok, frame = cap.read()
    frame = picam2.capture_array()
    # frame = np.array(np_array)

    # print(f"YUV {frame.shape}")

    if frame is None:
        time.sleep(0.01)
        continue

    # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.cvtColor(frame, cv2.COLOR_YUV2GRAY_I420)
    # frame = cv2.cvtColor(frame, cv2.COLOR_YUV2RGB_I420)
    frame = cv2.resize(frame, (640,480))

    if len(frame.shape) == 3:
        h,w,ch = frame.shape
        fmt = PIXELFORMAT_UNCOMPRESSED_R8G8B8
        # image = Image(frame.data, w,h,ch,PIXELFORMAT_UNCOMPRESSED_R8G8B8)
        # image = Image(frame, w,h,ch,PIXELFORMAT_UNCOMPRESSED_R8G8B8)
    else:
        h,w = frame.shape
        ch = 1
        fmt = PIXELFORMAT_UNCOMPRESSED_GRAYSCALE
        # image = Image(frame, w,h,1,PIXELFORMAT_UNCOMPRESSED_GRAYSCALE)

    # print(f"BGR {w}x{h}x{ch}")

    image = Image(frame,w,h,ch,fmt)
    texture = load_texture_from_image(image)
    # unload_image(image)

    begin_drawing()
    clear_background(BLACK)

    draw_texture(texture, 0, 0, WHITE)

    end_drawing()
    unload_texture(texture)
    # unload_image(image)

picam2.close()
close_window()