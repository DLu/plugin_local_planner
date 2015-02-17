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

#include <plp_extra_plugins/path_orientation_cost_function.h>
#include <angles/angles.h>
using plugin_local_planner::Trajectory;

namespace plp_extra_plugins {

void PathOrientationCostFunction::initialize(std::string name, plugin_local_planner::LocalPlannerUtil *planner_util) {
    TrajectoryCostFunction::initialize(name, planner_util);

    ros::NodeHandle nh("~/" + name_);
    nh.param("front_offset_angle", front_offset_angle_, 0.0);
    nh.param("max_trans_angle", max_trans_angle_, M_PI);
    // TODO: Give the option to turn if the CURRENT angle is less than max_trans_angle
}


double PathOrientationCostFunction::scoreTrajectory(Trajectory &traj) {
  if(traj.getPointsSize()==0)
    return 0.0;

  double px, py, pth;
  traj.getPoint(traj.getPointsSize()-1, px, py, pth);

  unsigned int cell_x, cell_y;
  //we won't allow trajectories that go off the map... shouldn't happen that often anyways
  if ( ! costmap_->worldToMap(px, py, cell_x, cell_y)) {
      //we're off the map
      ROS_WARN("Off Map %f, %f", px, py);
      return -4.0;
  }

  unsigned int path_index = map_(cell_x, cell_y).index;
  if(path_index>=yaws_.size())
   return 0.0;
  double diff = fabs(angles::shortest_angular_distance(pth+front_offset_angle_, yaws_[path_index]));
  if(diff > max_trans_angle_ && (traj.xv_ > 0.0 || traj.yv_ > 0.0))
    return -1.0;
  else
    return diff;
}

void PathOrientationCostFunction::setGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan, double goal_x, double goal_y)
{
  target_poses_ = orig_global_plan;
  if(target_poses_.size()==0)
      return;
  yaws_.clear();
  
  for(unsigned int i=0; i<target_poses_.size();i++){
    double angle = tf::getYaw(target_poses_[i].pose.orientation);
    yaws_.push_back(angle);
  }
 
  map_.reset();  
  map_.setTargetCells(*costmap_, target_poses_);
}

} /* namespace plp_extra_plugins */

PLUGINLIB_EXPORT_CLASS(plp_extra_plugins::PathOrientationCostFunction, plugin_local_planner::TrajectoryCostFunction)
