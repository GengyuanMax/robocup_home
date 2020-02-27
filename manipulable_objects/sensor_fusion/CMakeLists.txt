cmake_minimum_required(VERSION 2.8.3)
project(sensor_fusion)
add_compile_options(-std=c++11)

## - - find packages - - 
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  roslib 
  std_msgs 
  std_srvs 
  sensor_msgs 
  image_transport 
  cv_bridge
  rhome_srvs
)

find_package(OpenCV  REQUIRED)
find_package(Boost 1.4.0 COMPONENTS filesystem REQUIRED)
find_package(PCL 1.7 REQUIRED)

set(VERSION_FILE_PATH "${CATKIN_DEVEL_PREFIX}/include")
file(MAKE_DIRECTORY "${VERSION_FILE_PATH}")

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp roslib std_msgs std_srvs sensor_msgs image_transport cv_bridge  rhome_srvs 
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
#  ${FLANN_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(sensor_fusion src/sensor_fusion.cpp )
add_dependencies(sensor_fusion ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor_fusion ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})