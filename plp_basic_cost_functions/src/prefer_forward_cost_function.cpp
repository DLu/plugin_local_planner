/*
 * prefer_forward_cost_function.cpp
 *
 *  Created on: Apr 4, 2012
 *      Author: tkruse
 */

#include <plp_basic_cost_functions/prefer_forward_cost_function.h>

#include <math.h>

PLUGINLIB_EXPORT_CLASS(plp_basic_cost_functions::PreferForwardCostFunction, plugin_local_planner::TrajectoryCostFunction)


using plugin_local_planner::Trajectory;

namespace plp_basic_cost_functions {

void PreferForwardCostFunction::initialize(std::string name, plugin_local_planner::LocalPlannerUtil *planner_util)
{
    plugin_local_planner::TrajectoryCostFunction::initialize(name, planner_util);
    penalty_ = 1.0;
}


double PreferForwardCostFunction::scoreTrajectory(Trajectory &traj) {
  // backward motions bad on a robot without backward sensors
  if (traj.xv_ < 0.0) {
    return penalty_;
  }
  // strafing motions also bad on such a robot
  if (traj.xv_ < 0.1 && fabs(traj.thetav_) < 0.2) {
    return penalty_;
  }
  // the more we rotate, the less we progress forward
  return fabs(traj.thetav_) * 10;
}

} /* namespace plp_basic_cost_functions */
