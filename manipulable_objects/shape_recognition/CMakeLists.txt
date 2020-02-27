cmake_minimum_required(VERSION 2.8.3)
project(shape_recognition)


## - - find packages - - 
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  roslib 
  std_msgs 
  std_srvs 
  sensor_msgs 
  image_transport 
  cv_bridge
  pcl_ros
  pcl_conversions
  rhome_srvs
  object_pose_estimation
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
  CATKIN_DEPENDS roscpp roslib std_msgs std_srvs sensor_msgs image_transport cv_bridge pcl_ros rhome_srvs object_pose_estimation
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


add_executable(shape_recognition src/shape_recognition.cpp )
add_dependencies(shape_recognition ${catkin_EXPORTED_TARGETS})
target_link_libraries(shape_recognition ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES})