![](pics/jane-foster.jpg)

# Mjolnir

Trying to develop some interesting robotic camera/software solutions.

**Under Development**

- EX8029
    - **Discontinued**, Linux never worked correctly but macOS worked nicely
    - sparkfun: [EX8029 stereo camera](https://www.sparkfun.com/products/14726)
        - macOS USB3: 720x2560 (left 720x1280, right 720x1280)
        - rolling shutter
- Picamera
    - Waveshare CM4-IO-A board with 2 picameras (rolling shutter)
    - `piOS Lite` Bullseye, 64-bit
    - CM4 seemed to die on me ... can't get another one, stupid Pi Foundation
- Arducam Global Shutter (single camera)
- Inertial Sensors
    - Adafruit: [LSM6DSOX + LIS3MDL 9DOF IMU](https://www.adafruit.com/product/4517)
    - Adafruit: [BMP390 - Precision Barometric Pressure and Altimeter](https://www.adafruit.com/product/4816)
        - https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp390l-ds001.pdf

## Interesting SW

- [opencv_camera](https://pypi.org/project/opencv-camera/)
- [moms_apriltag](https://pypi.org/project/moms-apriltag/)
- Others
    - https://github.com/alsora/ORB_SLAM2
    - https://github.com/uoip/stereo_ptam
    - https://github.com/petercorke/machinevision-toolbox-matlab
    - https://www.mdpi.com/2218-6581/9/3/56/htm
    - https://github.com/alspitz/cpu_monitor


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
