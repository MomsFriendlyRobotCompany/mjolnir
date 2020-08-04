#!/usr/bin/env python

import time
from serial import Serial
import struct

port = "/dev/tty.usbmodem144401"
speed = 115200

last = time.monotonic()

s = Serial(port, speed, timeout=0.01)
loop = 1
rate = 0.0
data_size = 10*4

try:
    while True:
        s.reset_input_buffer()
        s.write(b"g")
        bad = True
        for _ in range(40):
            m = s.read(1)
            if m == b"":
                print(".", end="", flush=True)
                # continue
            if m != b"\xff":
                print("x", end="", flush=True)
                # continue
            else:
                bad = False
                break
        if bad:
            continue
        d = s.read(data_size)
        num = len(d)
        while num != data_size:
            d += s.read(num-len(d))

        msg = struct.unpack("ffffffffff", d)
        a = msg[:3]   # g's
        g = msg[3:6]  # rads/sec
        m = msg[6:9]  # normalized to uTesla
        t = msg[9]    # C
        t = t*9/5+32  # F
        print(f"R: {rate:5.1f} A: {a[0]:5.2f} {a[1]:5.2f} {a[2]:5.2f} G: {g[0]:5.2f} {g[1]:5.2f} {g[2]:5.2f} M: {m[0]:5.1f} {m[1]:5.1f} {m[2]:5.1f} T: {t:3.1f}", end="\r")

        if loop % 100 == 0:
            now = time.monotonic()
            rate = 100/(now - last)
            last = now

        loop += 1
except KeyboardInterrupt:
    s.close()
    print("\n\nbye ...\n")
