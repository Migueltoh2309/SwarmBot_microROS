cmake_minimum_required(VERSION 3.8)
project(swarmbot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/imu_orientation_tf.py scripts/tune_wheel_pi.py scripts/test_wheel_ramp.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
