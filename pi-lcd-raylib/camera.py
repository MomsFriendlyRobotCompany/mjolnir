#!/usr/bin/env python3
from pyray import *
from enum import IntEnum
import cv2
import time


# Initialize the window
init_window(800, 600, "My 2D Window")

# Set the target frames per second
set_target_fps(60)

cap = cv2.VideoCapture(0)

# Game loop
while not window_should_close():
    ok, frame = cap.read()

    if not ok:
        time.sleep(0.01)
        continue

    # # Convert image to Raylib texture
    # image = Image(frame.data, frame.shape[1], frame.shape[0], 3, 4)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.resize(frame, (800,600))
    h,w = frame.shape[:2]
    # image = Image(frame.data, w,h,3,PIXELFORMAT_UNCOMPRESSED_R8G8B8)
    # image = Image(frame, w,h,3,PIXELFORMAT_UNCOMPRESSED_R8G8B8)
    image = Image(frame, w,h,1,PIXELFORMAT_UNCOMPRESSED_GRAYSCALE)
    texture = load_texture_from_image(image)
    # unload_image(image)

    begin_drawing()
    clear_background(RAYWHITE)

    draw_texture(texture, 0, 0, WHITE)

    end_drawing()
    unload_texture(texture)
    # unload_image(image)

close_window()