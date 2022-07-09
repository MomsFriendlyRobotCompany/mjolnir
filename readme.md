![](pics/jane-foster.jpg)

# Mjolnir

**Under Development**

- sparkfun: [EX8029 stereo camera](https://www.sparkfun.com/products/14726)
    - macOS USB3: 720x2560 (left 720x1280, right 720x1280)
- adafruit: [LSM6DSOX + LIS3MDL 9DOF IMU](https://www.adafruit.com/product/4517)
- adafruit: [LPS22 pressure sensor](https://www.adafruit.com/product/4633) not using because it doesn't seem to work reliably
- https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390l-ds001.pdf

## Interesting SW

- https://github.com/alsora/ORB_SLAM2
- https://github.com/uoip/stereo_ptam
- https://github.com/petercorke/machinevision-toolbox-matlab
- https://www.mdpi.com/2218-6581/9/3/56/htm
- https://github.com/alspitz/cpu_monitor

```
 |-----+--------------+--------------------------------------------------------------|
 | No. | Layer        | Description ( in simple terms and not academic in nature )   |
 |-----+--------------+--------------------------------------------------------------|
 | 7   | Application  | High level API(e.g HTTP, Websocket)                          |
 |-----+--------------+--------------------------------------------------------------|
 | 6   | Presentation | This is where the data form the network                      |
 |     |              | is translated to the application (encoding,                  |
 |     |              | compression, encryption). This is where TLS lives.           |
 |-----+--------------+--------------------------------------------------------------|
 | 5   | Session      | Where the sessions are established, think Sockets.           |
 |-----+--------------+--------------------------------------------------------------|
 | 4   | Transport    | Provides the means to send variable length data sequences.   |
 |     |              | Think TCP, UDP.                                              |
 |-----+--------------+--------------------------------------------------------------|
 | 3   | Network      | Provides the capability to send data sequences between       |
 |     |              | different networks. Think of routing of datagrams.           |
 |-----+--------------+--------------------------------------------------------------|
 | 2   | Data link    | This layer is in charge of the node to node data transfer.   |
 |     |              | Directly connected nodes.                                    |
 |-----+--------------+--------------------------------------------------------------|
 | 1   | Physical     | In this layer data is transmitted/received to/from a         |
 |     |              | physical device.                                             |
 |-----+--------------+--------------------------------------------------------------|
```


# MIT License

**Copyright (c) 2020 Kevin J. Walchko**

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
