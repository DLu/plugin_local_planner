#ifndef GOAL_DIST_COST_FUNCTION_H_
#define GOAL_DIST_COST_FUNCTION_H_

#include <plp_basic_cost_functions/map_grid_cost_function.h>

namespace plp_basic_cost_functions {

class GoalDistCostFunction: public MapGridCostFunction {
public:
  bool prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec);
};

} /* namespace plp_basic_cost_functions */
#endif /* GOAL_DIST_COST_FUNCTION_H_ */
