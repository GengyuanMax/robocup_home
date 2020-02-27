cmake_minimum_required(VERSION 2.8.3)
project(rhome_srvs)

## - - find packages - - 
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
  std_srvs
  rhome_msgs
)


set(VERSION_FILE_PATH "${CATKIN_DEVEL_PREFIX}/include")
file(MAKE_DIRECTORY "${VERSION_FILE_PATH}")

# Generate services in the 'srv' folder
add_service_files(
  FILES
  Onoff.srv
  Queryobject.srv
  Getinformation.srv
  Recognize.srv
  Position3d.srv
)


generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
  rhome_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp roslib std_msgs std_srvs sensor_msgs image_transport cv_bridge  rhome_msgs
)
