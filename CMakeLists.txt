cmake_minimum_required(VERSION 3.8)
project(robot_phase1_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  docs
  launch
  maps
  repos
  rviz
  udev
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS
  scripts/setup_rock5c_jazzy.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
