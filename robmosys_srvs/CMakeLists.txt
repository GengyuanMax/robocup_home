cmake_minimum_required(VERSION 2.8.3)
project(robmosys_srvs)


## - - find packages - - 
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
  std_srvs
)



set(VERSION_FILE_PATH "${CATKIN_DEVEL_PREFIX}/include")
file(MAKE_DIRECTORY "${VERSION_FILE_PATH}")

add_message_files(
  FILES
  color.msg
  shape.msg
)


# Generate services in the 'srv' folder
add_service_files(
  FILES
  objectinformation.srv
)


generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp roslib std_msgs std_srvs sensor_msgs image_transport cv_bridge 
)
