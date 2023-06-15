# Install script for directory: /home/nagy/Desktop/software_engineering_robotics_Udacity/projects/catkin_ws_chase_ball/src/ball_chaser

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/nagy/Desktop/software_engineering_robotics_Udacity/projects/catkin_ws_chase_ball/install")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ball_chaser/srv" TYPE FILE FILES "/home/nagy/Desktop/software_engineering_robotics_Udacity/projects/catkin_ws_chase_ball/src/ball_chaser/srv/DriveToTarget.srv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ball_chaser/cmake" TYPE FILE FILES "/home/nagy/Desktop/software_engineering_robotics_Udacity/projects/catkin_ws_chase_ball/build/ball_chaser/catkin_generated/installspace/ball_chaser-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/nagy/Desktop/software_engineering_robotics_Udacity/projects/catkin_ws_chase_ball/devel/include/ball_chaser")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/nagy/Desktop/software_engineering_robotics_Udacity/projects/catkin_ws_chase_ball/devel/share/roseus/ros/ball_chaser")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/nagy/Desktop/software_engineering_robotics_Udacity/projects/catkin_ws_chase_ball/devel/share/common-lisp/ros/ball_chaser")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/nagy/Desktop/software_engineering_robotics_Udacity/projects/catkin_ws_chase_ball/devel/share/gennodejs/ros/ball_chaser")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/nagy/Desktop/software_engineering_robotics_Udacity/projects/catkin_ws_chase_ball/devel/lib/python3/dist-packages/ball_chaser")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/nagy/Desktop/software_engineering_robotics_Udacity/projects/catkin_ws_chase_ball/devel/lib/python3/dist-packages/ball_chaser")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/nagy/Desktop/software_engineering_robotics_Udacity/projects/catkin_ws_chase_ball/build/ball_chaser/catkin_generated/installspace/ball_chaser.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ball_chaser/cmake" TYPE FILE FILES "/home/nagy/Desktop/software_engineering_robotics_Udacity/projects/catkin_ws_chase_ball/build/ball_chaser/catkin_generated/installspace/ball_chaser-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ball_chaser/cmake" TYPE FILE FILES
    "/home/nagy/Desktop/software_engineering_robotics_Udacity/projects/catkin_ws_chase_ball/build/ball_chaser/catkin_generated/installspace/ball_chaserConfig.cmake"
    "/home/nagy/Desktop/software_engineering_robotics_Udacity/projects/catkin_ws_chase_ball/build/ball_chaser/catkin_generated/installspace/ball_chaserConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ball_chaser" TYPE FILE FILES "/home/nagy/Desktop/software_engineering_robotics_Udacity/projects/catkin_ws_chase_ball/src/ball_chaser/package.xml")
endif()

