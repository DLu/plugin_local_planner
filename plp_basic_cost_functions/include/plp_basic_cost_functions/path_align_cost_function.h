#ifndef PATH_ALIGN_COST_FUNCTION_H_
#define PATH_ALIGN_COST_FUNCTION_H_

#include <plp_basic_cost_functions/offset_grid_cost_function.h>

namespace plp_basic_cost_functions {

class PathAlignCostFunction: public OffsetGridCostFunction {
public:
  void initialize(std::string name, plugin_local_planner::LocalPlannerUtil *planner_util);
  bool prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec);
};

} /* namespace plp_basic_cost_functions */
#endif /* PATH_ALIGN_COST_FUNCTION_H_ */
