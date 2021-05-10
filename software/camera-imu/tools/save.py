import pickle
from collections import namedtuple
from collections import deque
from .imu_driver_alt import AccelGyroMag, TempPress, Light


def to_pickle(data, filename):
    with open(filename, 'wb') as fd:
        d = pickle.dumps(data)
        fd.write(d)

    print(f">> Saved {len(data)} data points to {filename}")

def from_pickle(filename):
    with open(filename, 'rb') as fd:
        data = pickle.load(fd)

    print(f">> Loaded {len(data)} data points from {filename}")
    return data
