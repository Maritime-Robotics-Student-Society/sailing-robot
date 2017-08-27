This folder contains scripts for communicating with the raspis we use.

First, source one of the pi identity files, which set $SAIL_PI_IP:

    source piaccess/magellan
    # OR
    source piaccess/zhenghe

Then run:

- `piaccess/ssh` : SSH to the raspi
- `piaccess/time2pi.sh` : Set the raspi's clock from your computer
- `piaccess/push2pi.sh` : Push this git repository to the raspi
- `piaccess/check_compass_calib.py` : Plot the latest compass calibration data
