#ifndef GOAL_ALIGN_COST_FUNCTION_H_
#define GOAL_ALIGN_COST_FUNCTION_H_

#include <plp_basic_cost_functions/offset_grid_cost_function.h>

namespace plp_basic_cost_functions {

class GoalAlignCostFunction: public OffsetGridCostFunction {
public:
  void initialize(std::string name, plugin_local_planner::LocalPlannerUtil *planner_util);
  bool prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec);

  virtual void setGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan, double goal_x, double goal_y);

private:
    tf::Stamped<tf::Pose> global_pose_;
    double goal_x_, goal_y_;

};

} /* namespace plp_basic_cost_functions */
#endif /* GOAL_ALIGN_COST_FUNCTION_H_ */
