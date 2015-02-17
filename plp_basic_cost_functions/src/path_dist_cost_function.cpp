#include <plp_basic_cost_functions/path_dist_cost_function.h>

PLUGINLIB_EXPORT_CLASS(plp_basic_cost_functions::PathDistCostFunction, plugin_local_planner::TrajectoryCostFunction)

using plugin_local_planner::Trajectory;

namespace plp_basic_cost_functions {

bool PathDistCostFunction::prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec) {
  map_.resetPathDist();
  map_.setTargetCells(*costmap_, target_poses_);
  return true;
}

} /* namespace plp_basic_cost_functions */

 
