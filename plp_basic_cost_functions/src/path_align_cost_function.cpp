#include <plp_basic_cost_functions/path_align_cost_function.h>

PLUGINLIB_EXPORT_CLASS(plp_basic_cost_functions::PathAlignCostFunction, plugin_local_planner::TrajectoryCostFunction)

using plugin_local_planner::Trajectory;

namespace plp_basic_cost_functions {


void PathAlignCostFunction::initialize(std::string name, plugin_local_planner::LocalPlannerUtil *planner_util)
{
    MapGridCostFunction::initialize(name, planner_util);
    stop_on_failure_ = false;

    ros::NodeHandle nh("~/" + name_);
    std::string key;
    if (nh.searchParam("forward_point_distance", key))
    {
        nh.getParam(key, xshift_);
        yshift_ = 0.0;
        shift_d_ = xshift_;
    }else{
        xshift_ = 0.325;
        yshift_ = 0.0;
        shift_d_ = xshift_;
    }

    quit_within_radius_ = true;
}

bool PathAlignCostFunction::prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec) {
  map_.resetPathDist();
  map_.setTargetCells(*costmap_, target_poses_);
  return true;
}

} /* namespace plp_basic_cost_functions */

 
