cmake_minimum_required(VERSION 2.8.3)
project(plugin_local_planner)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(
    cfg/PluginPlanner.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES plugin_local_planner
  CATKIN_DEPENDS costmap_2d dynamic_reconfigure nav_core nav_msgs pluginlib roscpp tf
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(local_planner_core
	src/goal_functions.cpp
	src/latched_stop_rotate_controller.cpp
	src/local_planner_util.cpp
	src/map_cell.cpp
	src/map_grid.cpp
	src/map_grid_visualizer.cpp
	src/odometry_helper_ros.cpp
	src/scale_manager.cpp
	src/simple_scored_sampling_planner.cpp
	src/simple_trajectory_generator.cpp
	src/trajectory.cpp
)

add_library(plugin_planner
    src/plugin_planner.cpp
    src/plugin_planner_ros.cpp
)

target_link_libraries(plugin_planner
  local_planner_core
  ${catkin_LIBRARIES}
)

add_dependencies(plugin_planner ${PROJECT_NAME}_gencfg)


add_executable(standalone_planner src/standalone_planner.cpp)

## Declare a cpp library
# add_library(plugin_local_planner
#   src/${PROJECT_NAME}/plugin_local_planner.cpp
# )

## Declare a cpp executable
# add_executable(plugin_local_planner_node src/plugin_local_planner_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(plugin_local_planner_node plugin_local_planner_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(standalone_planner
  plugin_planner 
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS plugin_local_planner plugin_local_planner_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

