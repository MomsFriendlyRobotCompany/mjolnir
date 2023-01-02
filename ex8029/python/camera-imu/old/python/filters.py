
from collections import deque
from collections import namedtuple


class AverageFilter(deque):
    def __init__(self, maxlen=5):
        super().__init__(maxlen=maxlen)
        for i in range(maxlen):
            # self.__setitem__(i, 0.0)
            self.append(np.zeros(3))

    def avg(self):
        avg = 0
        num = self.__len__()
        # print(num)
        for i in range(num):
            # print(self.__getitem__(i), end=" ")
            avg += self.__getitem__(i)
        return avg/num

def normalize3(x, y, z):
    """Return a unit vector"""
    norm = sqrt(x * x + y * y + z * z)

    # already a unit vector
    if norm == 1.0:
        return (x, y, z)

    inorm = 1.0/norm
    if inorm > 1e-6:
        x *= inorm
        y *= inorm
        z *= inorm
    else:
        raise ZeroDivisionError(f'norm({x:.4f}, {y:.4f}, {z:.4f},) = {inorm:.6f}')
    return (x, y, z,)
