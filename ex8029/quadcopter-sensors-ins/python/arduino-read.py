#!/usr/bin/env python3

# from yivo.serialport import SerialPort, findSerialPort
# import yivo
# from pprint import pprint
# from slurm import scistorage
# from slurm import files
import time
from tqdm import tqdm
from collections import deque
import pathlib
from serial import Serial
from slurm import storage
import sys

def find(path, fname):
    """Given a path, this will recursively search for a file (bob.txt) or
    pattern (\*.txt). It returns an array of found file paths."""
    fn = []
    for p in pathlib.Path(path).rglob(fname):
        fn.append(p)
    return fn

def findSerialPort():
    # handle macOS or Linux
    sp = find("/dev","tty.usbmodem*")[0].as_posix()
    # print(sp)
    return sp


def main():
    port = findSerialPort()
    print(f">> Using port {port}")

    s = Serial(port, 1000000, timeout=0.005)
    if not s.is_open:
            print("*** serial fail ***")
            sys.exit(1)

    data = []

    for cnt in tqdm(range(300)):
        s.write(b"g\n")
        s.flush()
        time.sleep(0.001)
        d = s.read(60)
        # print(d)
        if d is not None:
            # if d[:2] == [0xff,0xff]:
            data.append(d)

    s.close()

    if len(data) > 0:
        for d in data:
            print(f"-> {d}")

        storage.write("data.pickle", data)
    else:
        print("No data captured")

if __name__ == "__main__":
    main()
