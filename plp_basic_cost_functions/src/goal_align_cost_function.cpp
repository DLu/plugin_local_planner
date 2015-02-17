#include <plp_basic_cost_functions/goal_align_cost_function.h>

using plugin_local_planner::Trajectory;

PLUGINLIB_EXPORT_CLASS(plp_basic_cost_functions::GoalAlignCostFunction, plugin_local_planner::TrajectoryCostFunction)

namespace plp_basic_cost_functions {

void GoalAlignCostFunction::initialize(std::string name, plugin_local_planner::LocalPlannerUtil *planner_util)
{
    MapGridCostFunction::initialize(name, planner_util);
    stop_on_failure_ = false;
    ros::NodeHandle nh("~/" + name_);
    std::string key;
    if (nh.searchParam("forward_point_distance", key))
    {
        nh.getParam(key, xshift_);
        yshift_ = 0.0;
    }else{
        xshift_ = 0.325;
        yshift_ = 0.0;
    }
    shift_d_ = xshift_;
    
    quit_within_radius_ = false;
}

bool GoalAlignCostFunction::prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec) {
    map_.resetPathDist();
  
    global_pose_ = global_pose;
  
    double px = global_pose_.getOrigin().getX(), py = global_pose_.getOrigin().getY(), pth = tf::getYaw(global_pose_.getRotation());

    double sq_dist = (px - goal_x_) * (px - goal_x_) + (py - goal_y_) * (py - goal_y_);

    // we want the robot nose to be drawn to its final position
    // (before robot turns towards goal orientation), not the end of the
    // path for the robot center. Choosing the final position after
    // turning towards goal orientation causes instability when the
    // robot needs to make a 180 degree turn at the end
    double angle_to_goal = atan2(goal_y_ - py, goal_x_ - px);

    target_poses_.back().pose.position.x += xshift_ * cos(angle_to_goal);
    target_poses_.back().pose.position.y += xshift_ * sin(angle_to_goal);
  
  
    map_.setLocalGoal(*costmap_, target_poses_);
    return true;
}


void GoalAlignCostFunction::setGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan, double goal_x, double goal_y){
    goal_x_ = goal_x;
    goal_y_ = goal_y;
    target_poses_ = orig_global_plan;    
}

} /* namespace plp_basic_cost_functions */

 
