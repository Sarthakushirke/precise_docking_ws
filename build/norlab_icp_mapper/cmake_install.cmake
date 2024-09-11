# Install script for directory: /home/sarthak/precise_docking_ws/src/norlab_icp_mapper

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/sarthak/precise_docking_ws/install/norlab_icp_mapper")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/sarthak/precise_docking_ws/build/norlab_icp_mapper/libnorlab_icp_mapper.a")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/norlab_icp_mapper" TYPE FILE FILES
    "/home/sarthak/precise_docking_ws/src/norlab_icp_mapper/norlab_icp_mapper/Mapper.h"
    "/home/sarthak/precise_docking_ws/src/norlab_icp_mapper/norlab_icp_mapper/Map.h"
    "/home/sarthak/precise_docking_ws/src/norlab_icp_mapper/norlab_icp_mapper/Trajectory.h"
    "/home/sarthak/precise_docking_ws/src/norlab_icp_mapper/norlab_icp_mapper/CellManager.h"
    "/home/sarthak/precise_docking_ws/src/norlab_icp_mapper/norlab_icp_mapper/RAMCellManager.h"
    "/home/sarthak/precise_docking_ws/src/norlab_icp_mapper/norlab_icp_mapper/HardDriveCellManager.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/norlab_icp_mapper/MapperModules" TYPE FILE FILES
    "/home/sarthak/precise_docking_ws/src/norlab_icp_mapper/norlab_icp_mapper/MapperModules/MapperModule.h"
    "/home/sarthak/precise_docking_ws/src/norlab_icp_mapper/norlab_icp_mapper/MapperModules/DynamicPointsMapperModule.h"
    "/home/sarthak/precise_docking_ws/src/norlab_icp_mapper/norlab_icp_mapper/MapperModules/OctreeMapperModule.h"
    "/home/sarthak/precise_docking_ws/src/norlab_icp_mapper/norlab_icp_mapper/MapperModules/PointDistanceMapperModule.h"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xdevx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/norlab_icp_mapper" TYPE FILE FILES
    "/home/sarthak/precise_docking_ws/build/norlab_icp_mapper/CMakeFiles/norlab_icp_mapperConfig.cmake"
    "/home/sarthak/precise_docking_ws/build/norlab_icp_mapper/norlab_icp_mapperConfigVersion.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/sarthak/precise_docking_ws/build/norlab_icp_mapper/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
