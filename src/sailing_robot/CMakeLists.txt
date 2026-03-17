cmake_minimum_required(VERSION 3.8)
project(sailing_robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate ROS messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Velocity.msg"
  "msg/gpswtime.msg"
  "msg/BatteryState.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

# Install the Python package (sailing_robot library)
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR src/sailing_robot
)

# Install node scripts
install(PROGRAMS
  scripts/actuator_demand_rudder
  scripts/actuator_demand_sail
  scripts/actuator_driver_rudder
  scripts/actuator_driver_sail
  scripts/debugging_2D_plot
  scripts/debugging_2D_plot_matplot
  scripts/debugging_blink_on_sailing_state
  scripts/debugging_dashboard
  scripts/debugging_dump_params
  scripts/debugging_gps_log
  scripts/dummy_apparent_wind_direction
  scripts/dummy_heading
  scripts/dummy_position
  scripts/force_jibe_tack
  scripts/helming
  scripts/sensor_camera_detect
  scripts/sensor_driver_battery
  scripts/sensor_driver_gps
  scripts/sensor_driver_imu
  scripts/sensor_driver_imu_fusion
  scripts/sensor_driver_imu_without_cali
  scripts/sensor_driver_multiplexer
  scripts/sensor_driver_wind_direction
  scripts/sensor_processed_wind_direction
  scripts/sensor_service_imu
  scripts/simulation_gps_fix
  scripts/simulation_heading
  scripts/simulation_position
  scripts/simulation_velocity
  scripts/simulation_wind_apparent
  scripts/tack
  scripts/tasks
  scripts/wave_period
  scripts/wave_position
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install parameter files
install(DIRECTORY
  launch/parameters
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(sailing_robot_tests tests)
endif()

ament_package()
