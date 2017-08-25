The scripts in this directory calibrate sensors on the boat.

- `run_calibration.sh`: calibrates compass and wind direction sensors,
  dumps ROS parameters to a YAML file.
  - `compasscalib_roll`: calibrates the MinIMU compass, including roll compensation
  - `wind_direction_calib`: calibrates the wind direction sensor
  - See also `piaccess/check_compass_calib.py` to visualise the results of a
    compass calibration.
- `camera_detection_calibration.py`: calibrate the object recognition code
  using either a webcam or image files.

The Xsens compass has to be calibrated separately using a laptop.
