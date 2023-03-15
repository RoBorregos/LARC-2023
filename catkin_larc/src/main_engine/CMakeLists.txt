cmake_minimum_required(VERSION 3.0.2)
project(main_engine)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  actionlib
  actionlib_msgs
  geometry_msgs
  rospy
  std_msgs
  std_srvs
)


catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# install(PROGRAMS
#        scripts/main_engine.py
#        scripts/statemachine.py
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#) 