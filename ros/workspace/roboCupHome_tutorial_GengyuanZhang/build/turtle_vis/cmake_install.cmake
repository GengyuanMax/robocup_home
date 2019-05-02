# Install script for directory: /home/zhanggengyuan/robocup_home/ros/workspace/roboCupHome_tutorial_GengyuanZhang/src/turtle_vis

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zhanggengyuan/robocup_home/ros/workspace/roboCupHome_tutorial_GengyuanZhang/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtle_vis/msg" TYPE FILE FILES "/home/zhanggengyuan/robocup_home/ros/workspace/roboCupHome_tutorial_GengyuanZhang/src/turtle_vis/msg/DesiredPose.msg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtle_vis/srv" TYPE FILE FILES "/home/zhanggengyuan/robocup_home/ros/workspace/roboCupHome_tutorial_GengyuanZhang/src/turtle_vis/srv/send_desired_pose.srv")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtle_vis/cmake" TYPE FILE FILES "/home/zhanggengyuan/robocup_home/ros/workspace/roboCupHome_tutorial_GengyuanZhang/build/turtle_vis/catkin_generated/installspace/turtle_vis-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/zhanggengyuan/robocup_home/ros/workspace/roboCupHome_tutorial_GengyuanZhang/devel/include/turtle_vis")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/zhanggengyuan/robocup_home/ros/workspace/roboCupHome_tutorial_GengyuanZhang/devel/share/roseus/ros/turtle_vis")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/zhanggengyuan/robocup_home/ros/workspace/roboCupHome_tutorial_GengyuanZhang/devel/share/common-lisp/ros/turtle_vis")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/zhanggengyuan/robocup_home/ros/workspace/roboCupHome_tutorial_GengyuanZhang/devel/share/gennodejs/ros/turtle_vis")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/zhanggengyuan/robocup_home/ros/workspace/roboCupHome_tutorial_GengyuanZhang/devel/lib/python2.7/dist-packages/turtle_vis")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/zhanggengyuan/robocup_home/ros/workspace/roboCupHome_tutorial_GengyuanZhang/devel/lib/python2.7/dist-packages/turtle_vis")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zhanggengyuan/robocup_home/ros/workspace/roboCupHome_tutorial_GengyuanZhang/build/turtle_vis/catkin_generated/installspace/turtle_vis.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtle_vis/cmake" TYPE FILE FILES "/home/zhanggengyuan/robocup_home/ros/workspace/roboCupHome_tutorial_GengyuanZhang/build/turtle_vis/catkin_generated/installspace/turtle_vis-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtle_vis/cmake" TYPE FILE FILES
    "/home/zhanggengyuan/robocup_home/ros/workspace/roboCupHome_tutorial_GengyuanZhang/build/turtle_vis/catkin_generated/installspace/turtle_visConfig.cmake"
    "/home/zhanggengyuan/robocup_home/ros/workspace/roboCupHome_tutorial_GengyuanZhang/build/turtle_vis/catkin_generated/installspace/turtle_visConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtle_vis" TYPE FILE FILES "/home/zhanggengyuan/robocup_home/ros/workspace/roboCupHome_tutorial_GengyuanZhang/src/turtle_vis/package.xml")
endif()

