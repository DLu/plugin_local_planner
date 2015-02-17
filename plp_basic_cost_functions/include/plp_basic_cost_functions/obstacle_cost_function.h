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

#ifndef OBSTACLE_COST_FUNCTION_H_
#define OBSTACLE_COST_FUNCTION_H_

#include <plugin_local_planner/trajectory_cost_function.h>

#include <plp_basic_cost_functions/costmap_model.h>
#include <costmap_2d/costmap_2d.h>

namespace plp_basic_cost_functions {

/**
 * class ObstacleCostFunction
 * @brief Uses costmap 2d to assign negative costs if robot footprint
 * is in obstacle on any point of the trajectory.
 */
class ObstacleCostFunction : public plugin_local_planner::TrajectoryCostFunction {

public:
  ObstacleCostFunction(): world_model_(NULL) {}
  ~ObstacleCostFunction();

  virtual void initialize(std::string name, plugin_local_planner::LocalPlannerUtil *planner_util);
  bool prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec);
  double scoreTrajectory(plugin_local_planner::Trajectory &traj);

  // helper functions, made static for easy unit testing
  static double getScalingFactor(plugin_local_planner::Trajectory &traj, double scaling_speed, double max_trans_vel, double max_scaling_factor);
  static double footprintCost(
      const double& x,
      const double& y,
      const double& th,
      double scale,
      std::vector<geometry_msgs::Point> footprint_spec,
      costmap_2d::Costmap2D* costmap,
      plp_basic_cost_functions::WorldModel* world_model);
  virtual float getCost(unsigned int cx, unsigned int cy){ return costmap_->getCost(cx, cy); }

private:
  std::vector<geometry_msgs::Point> footprint_spec_;
  plp_basic_cost_functions::WorldModel* world_model_;
  bool sum_scores_;
  //footprint scaling with velocity;
  double max_scaling_factor_, scaling_speed_;
};

} /* namespace plugin_local_planner */
#endif /* OBSTACLE_COST_FUNCTION_H_ */
