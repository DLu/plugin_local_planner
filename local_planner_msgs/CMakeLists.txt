cmake_minimum_required(VERSION 2.8.3)
project(local_planner_msgs)

find_package(catkin REQUIRED COMPONENTS
   message_generation geometry_msgs
)

add_message_files(
  FILES
  CostFunctionCost.msg
  PlanCost.msg
  PlanningEvaluation.msg
)

catkin_package()
