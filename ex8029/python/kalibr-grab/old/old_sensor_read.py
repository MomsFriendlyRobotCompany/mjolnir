#!/usr/bin/env python3
from serial import Serial
import pickle
from collections import deque
from yivo import Yivo
from yivo.packet import ImuAGMQPT

data = deque()
motors = deque()

# def printIMU(msg):
#     print("-------------------")
#     for m in msg:
#         print(f"  {m}")

# def printPressure(msg):
#     print("-------------------")
#     print(f"> {msg[0]:.3f} C   {msg[1]:.3f} Pa")

# def saveMsg(msgID, msg):
#     if msgID == 0xD0:
#         data["imu"].append(msg)
#     elif msgIX == 0xD1:
#         data["press"].append(msg)

###############################################################
s = Serial()
s.port = "/dev/tty.usbmodem14601"
s.baud = 1000000
s.timeout = 1
s.open()

yivo = Yivo()

s.write(b't') # start data stream

try:
    while True:
        msg = yivo.read_packet(s)

        if msg is None:
            print("x",end="",flush=True)
        else:
            if type(msg) == ImuAGMQPT:
                print(".",end="",flush=True)
                data.append(msg)
            else:
                motors.append(msg)
                print("v",end="",flush=True)

except KeyboardInterrupt:
    print("ctrl-c")
except Exception as e:
    print(e)

finally:
    s.write(b't') # stop data stream
    s.close()

    if len(data) > 0:
        filename = "agmqpt.pkl"
        with open(filename,"wb") as fd:
            pickle.dump(data, fd)
        print(f">> Saved {len(data)} data points to {filename}")

        filename = "motors.pkl"
        with open(filename,"wb") as fd:
            pickle.dump(motors, fd)
        print(f">> Saved {len(motors)} data points to {filename}")
