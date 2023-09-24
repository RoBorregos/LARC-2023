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
  vision
  tf2
  tf2_ros
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  lineSensor.msg
)

add_service_files(
  FILES
    Intake.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs vision
  DEPENDS system_lib
  message_runtime 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(main_engine src/main_engine.cpp)
add_dependencies(main_engine vision_generate_messages_cpp)
target_link_libraries(main_engine ${catkin_LIBRARIES})

# install(PROGRAMS
#        scripts/main_engine.py
#        scripts/statemachine.py
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#) 