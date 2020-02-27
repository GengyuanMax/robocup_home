cmake_minimum_required(VERSION 2.8.3)
project(instance_recognition_3d)
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
  pcl_ros
  pcl_conversions
  rhome_srvs
  object_pose_estimation
)

find_package(OpenCV  REQUIRED)
find_package(Boost 1.4.0 COMPONENTS filesystem REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(HDF5 REQUIRED)
find_package(FLANN REQUIRED)

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
  ${FLANN_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(instance_recognition_3d src/instance_recognition_3d.cpp )
add_dependencies(instance_recognition_3d ${catkin_EXPORTED_TARGETS})
target_link_libraries(instance_recognition_3d ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})

add_executable(instance_training_3d src/instance_training_3d.cpp src/lrhome_3d.cpp )
add_dependencies(instance_training_3d ${catkin_EXPORTED_TARGETS})
target_link_libraries(instance_training_3d ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${OpenCV_LIBS} ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})