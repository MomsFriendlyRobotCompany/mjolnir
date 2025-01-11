#!/bin/bash

if [[ $EUID -eq 0 ]]; then
   echo "Do not run this as root"
   exit 1
fi

# sudo apt install libgl1 libegl1
sudo apt install libcap-dev rpicam-apps
sudo apt install lm-sensors i2c-tools

# I don't like this, it installs x11
sudo apt install python3-venv python3-pip python3-picamera2

# setup python env
mkdir ~/venvs
python3 -m venv venvs/py
sed -i 's/include-system-site-packages = false/include-system-site-packages = true/' ~/venvs/py/pyvenv.cfg
. ~/venvs/py/bin/activate
pip install -U pip setuptools
pip install -U raylib_drm opencv-python
