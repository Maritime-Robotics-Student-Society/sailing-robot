# Systems checklist

To ensure that things are running with real hardware. Setup steps:

1. Ensure RasPi and Arduinos have correct code loaded from Github, and the RasPi
   code is built (`catkin_make` in sailing-robot directory).
2. Connect RasPi to power source, and Arduinos to RasPi.
3. Plug in all sensors and actuators
4. On the RasPi, run `source devel/setup.bash`, and then
   `roslaunch sailing_robot with-real-hardware.launch`

Check that:

- All nodes come up successfully
- Values 0-360 are being published on `/heading`
- `/heading` changes appropriately when the IMU is turned
- Values 0-360 are being published on `/wind_direction_apparent`
- `/wind_direction_apparent` changes when the windvane is manually moved
- `/wind_speed_apparent` falls to 0 when anemometer still, rises when manually moved
- TODO: smoothing of wind direction (and speed?)
- `/position` gives a sensible latitude and longitude
- `/position` changes if the setup is moved some distance
- `/sailing_state` is being published: should change between 'normal',
  'tack_to_port_tack' and 'tack_to_stbd_tack' depending on heading, wind
  direction and next waypoint.
- `/goal_heading` is being published: points to next waypoint or at beating
  angle close to wind.
- `/tack_sail` is published (0)
- `/tack_rudder` is published (0, 90, -90)
- `/rudder_control` is published:
  - Same as `/tack_rudder` when `/sailing_state` is tack
  - To correct `/heading` towards `/goal_heading` otherwise
- `/sail_servo` is published: pulls sail in close to the wind, on both sides
- `/rudder_control` is affecting rudder servo
- `/sail_servo` is affecting sheet winch servo
