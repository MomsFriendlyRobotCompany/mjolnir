#!/usr/bin/env python3

import cv2
import pickle
import numpy as np
import time

method = cv2.optflow.calcOpticalFlowDenseRLOF
hsv = np.zeros_like((480,640,3))
hsv[..., 1] = 255

def calc(old_frame,new_frame):
    # mm = []
    params = []
    flow = method(old_frame, new_frame, None, *params)

    # Encoding: convert the algorithm's output into Polar coordinates
    mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
    # Use Hue and Saturation to encode the Optical Flow
    # hsv[..., 0] = ang * 180 / np.pi / 2
    # hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)
    # Convert HSV image into BGR for demo
    # bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    # cv2.imshow("frame", frame_copy)
    # cv2.imshow("optical flow", bgr)
    # k = cv2.waitKey(25) & 0xFF
    # if k == 27:
    #    break
    # mm.append(flow)
    # old_frame = new_frame
    return flow


with open("cam.pickle", 'rb') as fd:
    data = pickle.load(fd)

pics = []
last = data[0][2]
dts = []
for d,sz,dt in data:
    im = np.array(d, dtype=np.uint8)
    im.reshape(sz)
    im = cv2.resize(im, (640,480))
    im = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
    pics.append(im)
    dts.append(dt - last)
    last = dt


with open("smaller.pickle","wb") as fd:
    d = pickle.dumps(pics)
    fd.write(d)

# flow = []
# old = pics[0]
# for im in pics:
#     f = calc(old, im)
#     flow.append(f)
#     old = im

# title = f"{pics[0].shape} - {len(pics)}"
# for im, tt in zip(flow, dts):
#     cv2.imshow(title, im)
#     k = cv2.waitKey(30)
#     time.sleep(tt)

# cv2.destroyAllWindows()
