cmake_minimum_required(VERSION 2.8.3)
project(wrpps_ros)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  FILES
  IntensityModelParam.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

file(GLOB SCRIPT_PROGRAMS node_scripts/*.py)
catkin_install_python(PROGRAMS ${SCRIPT_PROGRAMS}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/node_scripts
)

install(DIRECTORY arduino launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
