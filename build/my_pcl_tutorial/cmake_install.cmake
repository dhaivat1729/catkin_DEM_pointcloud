# Install script for directory: /media/dhaivat666/e4187b00-2f8d-4c09-b475-5966c6563009/IIIT Lab work/Catkin_workspaces/catkin_ws_pointcloud/src/my_pcl_tutorial

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/media/dhaivat666/e4187b00-2f8d-4c09-b475-5966c6563009/IIIT Lab work/Catkin_workspaces/catkin_ws_pointcloud/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/media/dhaivat666/e4187b00-2f8d-4c09-b475-5966c6563009/IIIT Lab work/Catkin_workspaces/catkin_ws_pointcloud/build/my_pcl_tutorial/catkin_generated/installspace/my_pcl_tutorial.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/my_pcl_tutorial/cmake" TYPE FILE FILES
    "/media/dhaivat666/e4187b00-2f8d-4c09-b475-5966c6563009/IIIT Lab work/Catkin_workspaces/catkin_ws_pointcloud/build/my_pcl_tutorial/catkin_generated/installspace/my_pcl_tutorialConfig.cmake"
    "/media/dhaivat666/e4187b00-2f8d-4c09-b475-5966c6563009/IIIT Lab work/Catkin_workspaces/catkin_ws_pointcloud/build/my_pcl_tutorial/catkin_generated/installspace/my_pcl_tutorialConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/my_pcl_tutorial" TYPE FILE FILES "/media/dhaivat666/e4187b00-2f8d-4c09-b475-5966c6563009/IIIT Lab work/Catkin_workspaces/catkin_ws_pointcloud/src/my_pcl_tutorial/package.xml")
endif()

