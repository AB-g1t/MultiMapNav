cmake_minimum_required(VERSION 3.0.2)
project(multi_map_nav)

## Compile as C++14
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp actionlib actionlib_msgs
  move_base_msgs geometry_msgs std_msgs tf2_ros
  map_server amcl nav_msgs visualization_msgs
)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)

## Actions
add_action_files(
  DIRECTORY action
  FILES MultiMapNav.action
)
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp actionlib actionlib_msgs
    move_base_msgs geometry_msgs std_msgs tf2_ros
    map_server amcl nav_msgs visualization_msgs message_runtime
  LIBRARIES yaml-cpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${SQLITE3_INCLUDE_DIRS}
  include
)

## map_manager
add_executable(map_manager_node src/map_manager.cpp)
target_link_libraries(map_manager_node ${catkin_LIBRARIES})
add_dependencies(map_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## wormhole_detector
add_executable(wormhole_detector_node src/wormhole_detector.cpp src/wormhole_database.cpp)
target_link_libraries(wormhole_detector_node ${catkin_LIBRARIES} ${SQLITE3_LIBRARIES})
add_dependencies(wormhole_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## trajectory_manager
add_executable(trajectory_manager_node src/trajectory_manager.cpp src/wormhole_database.cpp)
target_link_libraries(trajectory_manager_node ${catkin_LIBRARIES} ${SQLITE3_LIBRARIES})
add_dependencies(trajectory_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## multimap_nav_server
add_executable(multimap_nav_server src/multimap_nav_server.cpp src/wormhole_database.cpp)
target_link_libraries(multimap_nav_server ${catkin_LIBRARIES} ${SQLITE3_LIBRARIES})
add_dependencies(multimap_nav_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## populate_warehouse_wormholes
add_executable(populate_warehouse_wormholes src/populate_warehouse_wormholes.cpp src/wormhole_database.cpp)
target_link_libraries(populate_warehouse_wormholes ${catkin_LIBRARIES} ${SQLITE3_LIBRARIES})
add_dependencies(populate_warehouse_wormholes ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Install launch files, config files, and worlds
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
  FILES_MATCHING PATTERN "*.world"
)

install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.pgm"
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  FILES_MATCHING PATTERN "*.rviz"
)
