#!/usr/bin/env python3

import numpy as np
import pickle

with open("data-3.pickle", "rb") as fd:
    data = pickle.load(fd)

n = []
for d in data:
    a = d[0]
    g = d[1]
    t = d[-1]
    ga = (t,) + g + a
    print(ga)
    n.append(ga)

nd = np.array(n, dtype=float)
print(nd.shape)
np.save("gyro-accel-3.npy", nd)
