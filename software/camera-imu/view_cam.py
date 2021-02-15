#!/usr/bin/env python3

import cv2
import pickle
import numpy as np
import time

# method = cv2.optflow.calcOpticalFlowDenseRLOF
# params = []

# method = cv2.calcOpticalFlowFarneback
# params = [0.5, 3, 15, 3, 5, 1.2, 0]

method = cv2.optflow.calcOpticalFlowSparseToDense
params = []

hsv = np.zeros((480,640,3),dtype=np.uint8)
hsv[..., 1] = 255

def calc(old_frame,new_frame, params):
    # mm = []
    # params = []
    flow = method(old_frame, new_frame, None, *params)

    # Encoding: convert the algorithm's output into Polar coordinates
    mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
    print(new_frame.shape,mag.shape, ang.shape)
    # Use Hue and Saturation to encode the Optical Flow
    hsv[..., 0] = ang * 180 / np.pi / 2
    hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
    # Convert HSV image into BGR for demo
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    # cv2.imshow("frame", frame_copy)
    cv2.imshow("optical flow", bgr)
    k = cv2.waitKey(1) & 0xFF
    if k == 27 or k == ord("q"):
       exit(0)
    # mm.append(flow)
    # old_frame = new_frame
    return flow


with open("smaller.pickle", 'rb') as fd:
    data = pickle.load(fd)

pics = []
# last = data[0][2]
dts = []
# for d,sz,dt in data:
for im in data:
    # im = np.array(d, dtype=np.uint8)
    # im.reshape(sz)
    # im = cv2.resize(im, (640,480))
    im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    pics.append(im)
    # dts.append(dt - last)
    # last = dt

flow = []
old = pics[0]
print("old",old.shape)
for im in pics[1:]:
    f = calc(old, im, params)
    # flow.append(f)
    old = im

# title = f"{pics[0].shape} - {len(pics)}"
# for im, tt in zip(flow, dts):
#     cv2.imshow(title, im)
#     k = cv2.waitKey(30)
#     time.sleep(tt)

# cv2.destroyAllWindows()
