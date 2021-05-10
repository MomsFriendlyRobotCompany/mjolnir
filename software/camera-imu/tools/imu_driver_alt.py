
from serial import Serial
import struct
from math import log10, sin, cos, acos, atan2, asin, pi, sqrt
import time
from collections import namedtuple
from colorama import Fore

# agmpt_t = namedtuple("agmpt_t", "accel gyro mag pressure temperature timestamp")
# ImageIMU = namedtuple("ImageIMU","image accel gyro temperature timestamp")
AccelGyroMag = namedtuple("AccelGyroMag", "ax ay az gx gy gz mx my mz")
TempPress = namedtuple("TempPress", "temperature pressure")
Light = namedtuple("Light", "lux")

c2f = lambda t: t*9/5+32

class cAccelGyroMag:
    """
    Accel: g's
    Gyro: rads/sec
    Mag: uT
    """
    header = 0xfd
    unpack = struct.Struct("<9f").unpack
    length = 9*4
    def astuple(self, data):
        return AccelGyroMag(*self.unpack(data))

class cAccelGyro:
    header = 0xfe
    unpack = struct.Struct("<6f").unpack
    length = 6*4
    def astuple(self, data):
        raise NotImplementedError()

class cMag:
    header = 0xfc
    unpack = struct.Struct("<3f").unpack
    length = 3*4
    def astuple(self, data):
        raise NotImplementedError()

class cTempPress:
    """
    Temperature: C
    Pressure: hPa
    """
    header = 0xfb
    unpack = struct.Struct("<ff").unpack
    length = 2*4
    def astuple(self, data):
        return TempPress(*self.unpack(data))

class cLight:
    header = 0xf9
    unpack = struct.Struct("f").unpack
    length = 1*4
    def astuple(self, data):
        return Light(*self.unpack(data))


class cIRCamera:
    header = 0xf8
    unpack = struct.Struct(f"<{32*24}f").unpack
    length = 32*24*4
    def astuple(self, data):
        raise NotImplementedError()

Key = {
    cAccelGyroMag.header: cAccelGyroMag(),
    cAccelGyro.header: cAccelGyro(),
    cMag.header: cMag(),
    cTempPress.header: cTempPress(),
    cLight.header: cLight(),
    cIRCamera.header: cIRCamera(),
}


class Parser:
    """
    [0xFF,0xFF]: start
    0xFE: accel, gyro
    0xFD: accel, gyro, mag
    0xFC: mag
    0xFB: temperature, pressure
    0xFA:
    0xF9: light
    0xF8: MLX90640 IR camera
    0xF7-0xF1: unused
    0xF0: position, velocity, quaternion
    [0xEE,0xEE]: end
    """
    header = b"\xff"
    ender = b"\xee"

    def decode(self, data):
        # print(f"{Fore.CYAN}[{len(data)}]{Fore.YELLOW}{data}{Fore.RESET}", flush=True)
        if data[-2:] != b"\xee\xee":
            print(f"{Fore.RED} ERROR: wrong message ending: {data[-2:]}{Fore.RESET}")
            return None

        size = len(data)

        i = 0
        ret = []
        while True:
            try:
                k = data[i]
                parse = Key[k]
            except Exception as e:
                print(e)
                print(f"{Fore.RED}** Invalid key: {hex(data[i])}{Fore.RESET}")
                return ret

            i += 1 # header

            if 0:
                d = parse.unpack(data[i:i+parse.length])
                ret += d
            else:
                d = parse.astuple(data[i:i+parse.length])
                ret.append(d)

            i += parse.length # message length

            if i == size-2: # \xee\xee
                break
        return ret


class IMUDriver:
    __slots__ = ["s", "decoder"]

    def __init__(self, port):
        # speed = 115200
        speed = 1000000
        self.s = Serial(port, speed, timeout=0.005)
        self.decoder = Parser()
        print(f">> IMUDriver opened {port}@{speed}")

    def close(self):
        self.s.close()

    def read(self, cmd=b'g'):
        """
        Return: array of data or None
        """
        self.s.reset_input_buffer()
        self.s.write(cmd)
        bad = True

        while self.s.out_waiting > 0:
            time.sleep(0.001)

        while self.s.in_waiting < 10:
            # print(".", end="", flush=True)
            time.sleep(0.001)
        # print(" ")

        a = self.s.read(1)
        b = self.s.read(1)
        success = False
        for _ in range(8):
            if a == b"\xff" and b == b"\xff":
                success = True
                break
            time.sleep(0.001)
            a = b
            b = self.s.read(1)

        if not success:
            print(f"{Fore.RED}** failed header **{Fore.RESET}")
            time.sleep(0.001)
            self.s.flushInput()
            return None

        data_size = ord(self.s.read(1))
        # print(f">> {Fore.BLUE}data size:{Fore.RESET} {data_size}", flush=True)
        data = self.s.read(data_size)
        ret = self.decoder.decode(data)
        ret.append(time.time())
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
