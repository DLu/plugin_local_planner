#include <plp_basic_cost_functions/goal_dist_cost_function.h>

using plugin_local_planner::Trajectory;

PLUGINLIB_EXPORT_CLASS(plp_basic_cost_functions::GoalDistCostFunction, plugin_local_planner::TrajectoryCostFunction)

namespace plp_basic_cost_functions {

bool GoalDistCostFunction::prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec) {
  map_.resetPathDist();
  map_.setLocalGoal(*costmap_, target_poses_);
  return true;
}
} /* namespace plp_basic_cost_functions */

 
