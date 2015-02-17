cmake_minimum_required(VERSION 2.8.3)
project(plp_extra_plugins)

find_package(catkin REQUIRED COMPONENTS
  plugin_local_planner plp_basic_cost_functions
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(${PROJECT_NAME} 
  src/minimize_y_cost_function.cpp
  src/minimize_turns_cost_function.cpp
  src/closest_target_point_map.cpp
  src/path_orientation_cost_function.cpp
  src/goal_orientation_cost_function.cpp
  src/speed_cost_function.cpp
  src/jerk_cost_function.cpp
  src/turn_first_cost_function.cpp
)

target_link_libraries(${PROJECT_NAME}  plugin_local_planner)


