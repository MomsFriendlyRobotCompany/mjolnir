
from serial import Serial
import struct
from math import log10, sin, cos, acos, atan2, asin, pi, sqrt
import time
from collections import namedtuple

agmpt_t = namedtuple("agmpt_t", "accel gyro mag pressure temperature timestamp")

lambda t: t*9/5+32

class AGT:
    size = 7
    header = b"\xfc"

    def decode(self, data):
        if len(data) != self.size*4:
            return None

        msg = struct.unpack("fffffff", d)
        a = msg[:3]   # g's
        g = msg[3:6]  # rads/sec
        t = msg[6]    # C
        return a,g,t

class AGMPT:
    size = 11
    header = b"\xff"

    def decode(self, data):
        if len(data) != self.size*4:
            return None

        msg = struct.unpack("fffffffffff", data)
        a = msg[:3]   # g's
        g = msg[3:6]  # rads/sec
        m = msg[6:9]  # normalized to uTesla
        p = msg[9]    # pressure hPa
        t = msg[10]   # C
        # t = t*9/5+32  # F
        return a,g,m,p,t

class IMUDriver:
    __slots__ = ["s", "decoder"]
    # s = attr.ib(default=None)
    # port = attr.ib()
    # speed
    def __init__(self, port, decoder):
        speed = 115200
        self.s = Serial(port, speed, timeout=0.005)
        self.decoder = decoder

    def close(self):
        self.s.close()

    # def read_buffer(self):

    def read(self):
        data_size = self.decoder.size*4
        # self.s.flushInput()
        self.s.write(b"g")
        # self.s.flushOutput()
        bad = True
        time.sleep(0.001)

        # read 2 times the data size looking for the
        # start character
        header = self.decoder.header
        for _ in range(50):
            m = self.s.read(1)
            if m == header:
                bad = False
                break

            self.s.write(b"g")
            time.sleep(0.001)

        if bad:
            print("** read fail **")
            return None
        data = self.s.read(data_size)
        num = len(data)
        while num != data_size:
            data += self.s.read(num-len(data))

        # print(f">> data[{len(data)}]: {data}")

        ret = self.decoder.decode(data)

        return ret

    def compensate(self, accel, mag=None):
        """
        """
        try:
            ax, ay, az = normalize3(*accel)

            pitch = asin(-ax)

            if abs(pitch) >= pi/2:
                roll = 0.0
            else:
                roll = asin(ay/cos(pitch))

            if mag:
                # mx, my, mz = mag
                mx, my, mz = normalize3(*mag)
                x = mx*cos(pitch)+mz*sin(pitch)
                y = mx*sin(roll)*sin(pitch)+my*cos(roll)-mz*sin(roll)*cos(pitch)
                heading = atan2(y, x)

                # wrap heading between 0 and 360 degrees
                if heading > 2*pi:
                    heading -= 2*pi
                elif heading < 0:
                    heading += 2*pi
            else:
                heading = None

            # if self.angle_units == Angle.degrees:
            # roll    *= RAD2DEG
            # pitch   *= RAD2DEG
            # heading *= RAD2DEG
            # elif self.angle_units == Angle.quaternion:
            #     return Quaternion.from_euler(roll, pitch, heading)

            return (roll, pitch, heading,)

        except ZeroDivisionError as e:
            print('Error', e)
            # if self.angle_units == Angle.quaternion:
            #     return Quaternion(1, 0, 0, 0)
            # else:
            return (0.0, 0.0, 0.0,)

    def height(self, p):
        """
        given pressure in hPa, returns altitude in meters.
        """
        h = (1 - pow(p / 1013.25, 0.190263)) * 44330.8
        return h
