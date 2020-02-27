cmake_minimum_required(VERSION 2.8.3)
project(face_detection)
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
find_package(PCL REQUIRED)

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
  ${openblas_INCLUDE_DIRS}
#  ${FLANN_INCLUDE_DIRS}
)

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})


add_executable(face_detection src/face_detection.cpp src/mtcnn.cpp src/network.cpp src/pBox.cpp )
add_dependencies(face_detection ${catkin_EXPORTED_TARGETS})
target_link_libraries(face_detection ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES} openblas)
