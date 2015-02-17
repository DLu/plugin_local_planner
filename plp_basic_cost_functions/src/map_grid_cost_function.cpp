/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: TKruse
 *********************************************************************/

#include <plp_basic_cost_functions/map_grid_cost_function.h>

using plugin_local_planner::Trajectory;

namespace plp_basic_cost_functions {

void MapGridCostFunction::initialize(std::string name, plugin_local_planner::LocalPlannerUtil *planner_util) {
    plugin_local_planner::TrajectoryCostFunction::initialize(name, planner_util);
    stop_on_failure_ = true;

    ros::NodeHandle nh("~/" + name_);
    std::string aggro_str;
    nh.param("aggregation_type", aggro_str, std::string("last"));
    if(aggro_str=="last") 
        aggregationType_ = Last;
    else if(aggro_str=="sum")
        aggregationType_ = Sum;
    else if(aggro_str=="product")
        aggregationType_ = Product;
        
    map_.sizeCheck(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
}

float MapGridCostFunction::getCost(unsigned int px, unsigned int py) {
  double grid_dist = map_(px, py).target_dist;
  return grid_dist;
}

double MapGridCostFunction::scoreTrajectory(Trajectory &traj) {
  double cost = 0.0;
  if (aggregationType_ == Product) {
    cost = 1.0;
  }
  double px, py, pth;
  double grid_dist;

  for (unsigned int i = 0; i < traj.getPointsSize(); ++i) {
    traj.getPoint(i, px, py, pth);
    grid_dist = scoreCell(px, py, pth);
    if(stop_on_failure_){
      if (grid_dist == map_.obstacleCosts()) {
        return -3.0;
      } else if (grid_dist == map_.unreachableCellCosts()) {
        return -2.0;
      }
    }

    switch( aggregationType_ ) {
    case Last:
      cost = grid_dist;
      break;
    case Sum:
      cost += grid_dist;
      break;
    case Product:
      if (cost > 0) {
        cost *= grid_dist;
      }
      break;
    }
  }

  double factor = costmap_->getResolution() * 0.5;
  return cost * factor;
}

double MapGridCostFunction::scoreCell(double px, double py, double pth) {
    unsigned int cell_x, cell_y;
    //we won't allow trajectories that go off the map... shouldn't happen that often anyways
    if ( ! costmap_->worldToMap(px, py, cell_x, cell_y)) {
      //we're off the map
      ROS_WARN("Off Map %f, %f", px, py);
      return -4.0;
    }
    return getCost(cell_x, cell_y);
}

void MapGridCostFunction::setGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan, double goal_x, double goal_y)
{
    target_poses_ = orig_global_plan;
    goal_x_ = goal_x;
    goal_y_ = goal_y;
}

} /* namespace plp_basic_cost_functions */
