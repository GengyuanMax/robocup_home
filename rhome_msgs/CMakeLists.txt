cmake_minimum_required(VERSION 2.8.3)
project(rhome_msgs)

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

# Generate services in the 'msg' folder
add_message_files(
  FILES
  Roi.msg
  Face.msg
  Rect.msg
)



# Generate added messages and services with any dependencies listed here
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
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    sensor_msgs
    std_msgs
    std_srvs
)
