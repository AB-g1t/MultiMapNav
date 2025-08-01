# Install script for directory: /home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/src/multi_map_nav

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_map_nav/action" TYPE FILE FILES "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/src/multi_map_nav/action/MultiMapNav.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_map_nav/msg" TYPE FILE FILES
    "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/devel/share/multi_map_nav/msg/MultiMapNavAction.msg"
    "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/devel/share/multi_map_nav/msg/MultiMapNavActionGoal.msg"
    "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/devel/share/multi_map_nav/msg/MultiMapNavActionResult.msg"
    "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/devel/share/multi_map_nav/msg/MultiMapNavActionFeedback.msg"
    "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/devel/share/multi_map_nav/msg/MultiMapNavGoal.msg"
    "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/devel/share/multi_map_nav/msg/MultiMapNavResult.msg"
    "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/devel/share/multi_map_nav/msg/MultiMapNavFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_map_nav/cmake" TYPE FILE FILES "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/build/multi_map_nav/catkin_generated/installspace/multi_map_nav-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/devel/include/multi_map_nav")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/devel/share/roseus/ros/multi_map_nav")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/devel/share/common-lisp/ros/multi_map_nav")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/devel/share/gennodejs/ros/multi_map_nav")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/devel/lib/python3/dist-packages/multi_map_nav")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/devel/lib/python3/dist-packages/multi_map_nav")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/build/multi_map_nav/catkin_generated/installspace/multi_map_nav.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_map_nav/cmake" TYPE FILE FILES "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/build/multi_map_nav/catkin_generated/installspace/multi_map_nav-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_map_nav/cmake" TYPE FILE FILES
    "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/build/multi_map_nav/catkin_generated/installspace/multi_map_navConfig.cmake"
    "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/build/multi_map_nav/catkin_generated/installspace/multi_map_navConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_map_nav" TYPE FILE FILES "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/src/multi_map_nav/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_map_nav/launch" TYPE DIRECTORY FILES "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/src/multi_map_nav/launch/" FILES_MATCHING REGEX "/[^/]*\\.launch$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_map_nav/config" TYPE DIRECTORY FILES "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/src/multi_map_nav/config/" FILES_MATCHING REGEX "/[^/]*\\.yaml$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_map_nav/worlds" TYPE DIRECTORY FILES "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/src/multi_map_nav/worlds/" FILES_MATCHING REGEX "/[^/]*\\.world$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_map_nav/maps" TYPE DIRECTORY FILES "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/src/multi_map_nav/maps/" FILES_MATCHING REGEX "/[^/]*\\.yaml$" REGEX "/[^/]*\\.pgm$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/multi_map_nav/rviz" TYPE DIRECTORY FILES "/home/ab/MultiMapNav/multi_map_navigation_deliverables_proper_20250801_220611/multi_map_nav_workspace/src/multi_map_nav/rviz/" FILES_MATCHING REGEX "/[^/]*\\.rviz$")
endif()

