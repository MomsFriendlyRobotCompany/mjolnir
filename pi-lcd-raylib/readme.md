# DRM

## Setup

```bash
sudo apt install libgl1 libegl1 lm-sensors i2c-tools
sudo apt install python3-venv python3-pip
```

```bash
mkdir ~/venvs
python3 -m venv venvs/py
. ~/venvs/py/bin/activate

pip install -U raylib_drm opencv-python
```

```ini
# /boot/firmware/config.txt
ignore_lcd=1 # removes legacy autodetection
dtoverlay=vc4-kms-v3d
dtoverlay=vc4-kms-dsi-7inch,disable_touch
# dtoverlay=vc4-kms-dsi-7inch,sizex=400,invx,invy
```

| DT parameter  | Action
|---------------|-----------------------------
| sizex         | Sets X resolution (default 800)
| sizey         | Sets Y resolution (default 480)
| invx          | Invert X coordinates
| invy          | Invert Y coordinates
| swapxy        | Swap X and Y coordinates
| disable_touch | Disables the touch overlay totally

```ini
# /boot/firmware/cmdline.txt 
# <rotation-value> can be: 0 90 180 270
video=DSI-1:800x480@60,rotate=<rotation-value>
```

## 7 Inch Screen

- 800 x 480 @ 60 fps and 24bit colort
- Multi-touch panel, 10 points
- I2C interface for backlighting and power
- [Buy](https://www.raspberrypi.com/products/raspberry-pi-touch-display/)
- [Docs](https://www.raspberrypi.com/documentation/accessories/display.html)
- [Product Brief](https://datasheets.raspberrypi.com/display/7-inch-display-product-brief.pdf)
- [Mechanical Drawing](https://datasheets.raspberrypi.com/display/7-inch-display-mechanical-drawing.pdf)

## Camera

```bash
$ rpicam-hello --list-cameras
Available cameras
-----------------
0 : imx219 [3280x2464 10-bit RGGB] (/base/soc/i2c0mux/i2c@1/imx219@10)
    Modes: 'SRGGB10_CSI2P' : 640x480 [206.65 fps - (1000, 752)/1280x960 crop]
                             1640x1232 [41.85 fps - (0, 0)/3280x2464 crop]
                             1920x1080 [47.57 fps - (680, 692)/1920x1080 crop]
                             3280x2464 [21.19 fps - (0, 0)/3280x2464 crop]
           'SRGGB8' : 640x480 [206.65 fps - (1000, 752)/1280x960 crop]
                      1640x1232 [83.70 fps - (0, 0)/3280x2464 crop]
                      1920x1080 [47.57 fps - (680, 692)/1920x1080 crop]
                      3280x2464 [21.19 fps - (0, 0)/3280x2464 crop]
```

- [Camera Datasheet](https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf)

## Software

- [Camera Software](https://www.raspberrypi.com/documentation/computers/camera_software.html)