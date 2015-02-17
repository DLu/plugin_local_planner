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

generate_dynamic_reconfigure_options(
    cfg/PluginPlanner.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES plugin_local_planner
  CATKIN_DEPENDS costmap_2d dynamic_reconfigure nav_core nav_msgs pluginlib roscpp tf
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(local_planner_core
	src/goal_functions.cpp
	src/latched_stop_rotate_controller.cpp
	src/local_planner_util.cpp
	src/odometry_helper_ros.cpp
	src/scale_manager.cpp
	src/simple_scored_sampling_planner.cpp
	src/simple_trajectory_generator.cpp
	src/trajectory.cpp
)

add_library(plugin_local_planner
    src/plugin_planner.cpp
    src/plugin_planner_ros.cpp
)

target_link_libraries(plugin_local_planner
  local_planner_core
  ${catkin_LIBRARIES}
)

add_dependencies(plugin_local_planner ${PROJECT_NAME}_gencfg)


add_executable(standalone_planner src/standalone_planner.cpp)

target_link_libraries(standalone_planner
  plugin_local_planner 
  ${catkin_LIBRARIES}
)

install(TARGETS plugin_local_planner standalone_planner
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


