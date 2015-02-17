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

#ifndef MAP_GRID_COST_FUNCTION_H_
#define MAP_GRID_COST_FUNCTION_H_

#include <plugin_local_planner/trajectory_cost_function.h>

#include <costmap_2d/costmap_2d.h>
#include <plp_basic_cost_functions/map_grid.h>

namespace plp_basic_cost_functions {

/**
 * when scoring a trajectory according to the values in mapgrid, we can take
 *return the value of the last point (if no of the earlier points were in
 * return collision), the sum for all points, or the product of all (non-zero) points
 */
enum CostAggregationType { Last, Sum, Product};

/**
 * This class provides cost based on a map_grid of a small area of the world.
 * The map_grid covers a the costmap, the costmap containing the information
 * about sensed obstacles. The map_grid is used by setting
 * certain cells to distance 0, and then propagating distances around them,
 * filling up the area reachable around them.
 *
 * The approach using grid_maps is used for computational efficiency, allowing to
 * score hundreds of trajectories very quickly.
 *
 * This can be used to favor trajectories which stay on a given path, or which
 * approach a given goal.
 * @param costmap_ros Reference to object giving updates of obstacles around robot
 * @param xshift where the scoring point is with respect to robot center pose
 * @param yshift where the scoring point is with respect to robot center pose
 * @param is_local_goal_function, scores for local goal rather than whole path
 * @param aggregationType how to combine costs along trajectory
 */
class MapGridCostFunction: public plugin_local_planner::TrajectoryCostFunction {
public:

  MapGridCostFunction() : stop_on_failure_(true), aggregationType_(Last) {}

  virtual void initialize(std::string name, plugin_local_planner::LocalPlannerUtil *planner_util);
  virtual bool prepare(tf::Stamped<tf::Pose> global_pose,
		       tf::Stamped<tf::Pose> global_vel,
		       std::vector<geometry_msgs::Point> footprint_spec) = 0;

  virtual double scoreTrajectory(plugin_local_planner::Trajectory &traj);
  virtual double scoreCell(double px, double py, double pth);

  // used for easier debugging
  virtual float getCost(unsigned int cx, unsigned int cy);

  virtual void setGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan, double goal_x, double goal_y);

  inline bool isValidCost(double cost)
  {
    return cost!=map_.obstacleCosts() && cost!=map_.unreachableCellCosts();
  }

protected:
  std::vector<geometry_msgs::PoseStamped> target_poses_;
  double goal_x_, goal_y_;
  bool stop_on_failure_;

  plp_basic_cost_functions::MapGrid map_;
  CostAggregationType aggregationType_;

};

} /* namespace plp_basic_cost_functions */
#endif /* MAP_GRID_COST_FUNCTION_H_ */
