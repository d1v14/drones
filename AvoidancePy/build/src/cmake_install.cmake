# Install script for directory: /home/dmitry/drones/AvoidancePy/src/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/dmitry/drones/AvoidancePy/install")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/dmitry/drones/AvoidancePy/build/src/catkin_generated/installspace/avoidance.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/avoidance/cmake" TYPE FILE FILES
    "/home/dmitry/drones/AvoidancePy/build/src/catkin_generated/installspace/avoidanceConfig.cmake"
    "/home/dmitry/drones/AvoidancePy/build/src/catkin_generated/installspace/avoidanceConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/avoidance" TYPE FILE FILES "/home/dmitry/drones/AvoidancePy/src/src/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/avoidance" TYPE PROGRAM FILES
    "/home/dmitry/drones/AvoidancePy/src/src/src/avoidance_node.py"
    "/home/dmitry/drones/AvoidancePy/src/src/src/avoidance/avoidance_system.py"
    "/home/dmitry/drones/AvoidancePy/src/src/src/avoidance/flight_commander.py"
    "/home/dmitry/drones/AvoidancePy/src/src/src/mapping/histogram2d.py"
    "/home/dmitry/drones/AvoidancePy/src/src/src/mapping/simple_grid_map.py"
    "/home/dmitry/drones/AvoidancePy/src/src/src/mapping/map_visualizer.py"
    "/home/dmitry/drones/AvoidancePy/src/src/src/planners/path_visualization.py"
    "/home/dmitry/drones/AvoidancePy/src/src/src/planners/planer_base.py"
    "/home/dmitry/drones/AvoidancePy/src/src/src/planners/vf2d_planner.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/avoidance" TYPE PROGRAM FILES "/home/dmitry/drones/AvoidancePy/build/src/catkin_generated/installspace/avoidance_node.py")
endif()

