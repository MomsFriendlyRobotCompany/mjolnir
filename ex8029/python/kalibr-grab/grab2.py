#!/usr/bin/env python3

'''
--------------------------------------------------
Image: 720x1280
Markers.checkerboard: (7, 10)
Intrinsic Camera Parameters
--------------------------------------------------
 [Camera 1]
  f(x,y): 1129.4 1127.4 px
  principlePoint(x,y): 640.8 357.0 px
  distCoeffs [ 1.39107250e-01 -5.25456587e-01 -1.30556613e-04  9.07833455e-04
  9.33127229e-01]
 [Camera 2]
  f(x,y): 1128.5 1127.2 px
  principlePoint(x,y): 652.1 354.6 px
  distCoeffs [ 1.28455592e-01 -3.69658433e-01  5.09480888e-04 -2.52525323e-04
  4.11286905e-01]
--------------------------------------------------
Extrinsic Camera Parameters
--------------------------------------------------
  R [[ 0.99999112 -0.00148358 -0.00394393]
 [ 0.0014944   0.99999512  0.00274168]
 [ 0.00393984 -0.00274755  0.99998846]]
  T[meter] [[-0.03134446]
 [-0.00025078]
 [-0.00159318]]
  E [[ 1.39283028e-06  1.59385949e-03 -2.46405370e-04]
 [-1.46967190e-03 -8.37569159e-05  3.13503789e-02]
 [ 2.03932801e-04 -3.13446763e-02 -8.69255963e-05]]
  F [[ 4.60170665e-08  5.27552785e-05 -2.80562760e-02]
 [-4.86110558e-05 -2.77543141e-06  1.20330732e+00]
 [ 2.48110107e-02 -1.20419456e+00  1.00000000e+00]]
'''
import numpy as np
import cv2
import time # monotonic_ns for timestamp
from datetime import datetime as dtime # naming directory
from yivo import Yivo
from yivo.packet import ImuAGT, MsgIDs
from serial import Serial
from slurm import SimpleProcess
from slurm import Rate
from colorama import Fore
from opencv_camera import ThreadedCamera
from kalibr import KalibrWriteData
import struct
from threading import Event, Thread

record = False
sd = KalibrWriteData()
run = True

def ser_func():
    try:
        s = Serial()
        s.port = "/dev/tty.usbmodem144101"
        s.baud = 1000000
        s.timeout = 0.01
        s.open()
    except:
        print(f"{Fore.RED} *** Couldn't open serial port: {s.port} ***{Fore.RESET}")
        return
    print(f"{Fore.GREEN}Serial port openned: {s.is_open} {s.port} @ {s.baud}{Fore.RESET}")

    yivo = Yivo()
    msg_size = struct.calcsize("<2chB7fLB") #FIXME

    r = Rate(100)

    while run:
        s.write(b'T\n')

        num = msg_size
        msg = b''
        retry = 5
        while num > 0:
            time.sleep(0.001)
            chunk = s.read(num)
            num -= len(chunk)
            msg += chunk
            retry -= 1
            if retry == 0:
                break

        if msg:
            err,msgid,data = yivo.unpack(msg)
            if (err == 0) and (msgid == MsgIDs.IMU_AGT):
                if record:
                    sd.pushImu(*data[3:6],*data[:3])

        r.sleep()


if __name__ == '__main__':
    count = 0

    cam = ThreadedCamera()
    cam.open(1, fmt=8) # 1 BGR, 8 grayscale
    print(cam)

    pser = Thread(target=ser_func)
    pser.daemon = True
    pser.start()

    epoch = 0

    rate = Rate(30)

    try:
        while True:
            frame = cam.frame
            if frame is None:
                time.sleep(0.5)
                continue

            if record:
                r,c = frame.shape[:2]
                edge = c//2
                img_l,img_r = frame[:,:edge], frame[:,edge:]
                sd.pushImage(img_l,img_r)
                rate.sleep()

            r,c = frame.shape[:2]
            showImg = cv2.resize(frame, (c//4,r//4))
            cv2.imshow("win", showImg)

            c = cv2.waitKey(1)
            if c == 27 or c == ord('q'):
                print(f"\n{Fore.CYAN}quitting ...{Fore.RESET}")
                break
            elif c == ord('c'):
                sd.pushImage(frame)
            elif c == ord('s'):
                cv2.imwrite("./save/" + str(count) + ".png", frame)
                count += 1
            elif c == ord('r'):
                if record: # stop
                    dt = time.time() - epoch
                    print(f"{Fore.CYAN}Captured")
                    print(f"  image: {len(sd.camera)} frames \t{len(sd.camera)/dt:.1f} fps")
                    print(f"    imu: {len(sd.imu)} readings \t{len(sd.imu)/dt:.1f} ips")
                    print(f"{Fore.RESET}")
                    record = False
                    color = Fore.YELLOW
                else: # start
                    sd.clear()
                    record = True
                    color = Fore.GREEN
                    epoch = time.time()

                print(f"{color}>> Recording data: {record}{Fore.RESET}")


    except KeyboardInterrupt:
        print(f"\n{Fore.CYAN}ctl-c ...{Fore.RESET}")

    finally:
        run = False
        time.sleep(0.25)
        # sd.write(f"./data/{dtime.now().strftime('%m.%d.%Y-%H-%M-%S')}")
        cv2.destroyAllWindows()
        pser.join(timeout=2.0)
        cam.close()
        print("bye ...")
