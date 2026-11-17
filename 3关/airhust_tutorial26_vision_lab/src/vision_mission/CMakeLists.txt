cmake_minimum_required(VERSION 3.0.2)
project(vision_mission)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
  cv_bridge
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/vision_mission.py
  scripts/mission_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts/detectors scripts/utils
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)