cmake_minimum_required(VERSION 2.8.3)
project(plp_basic_cost_functions)

find_package(catkin REQUIRED COMPONENTS
  plugin_local_planner
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES plp_basic_cost_functions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(${PROJECT_NAME} 
	src/map_cell.cpp
	src/map_grid.cpp
    src/map_grid_cost_function.cpp
    src/goal_dist_cost_function.cpp
    src/path_dist_cost_function.cpp
    src/offset_grid_cost_function.cpp
    src/goal_align_cost_function.cpp
    src/path_align_cost_function.cpp
    src/obstacle_cost_function.cpp
    src/oscillation_cost_function.cpp
    src/prefer_forward_cost_function.cpp
    src/costmap_model.cpp
) 
target_link_libraries(${PROJECT_NAME}  plugin_local_planner)

install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

