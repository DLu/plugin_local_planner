#ifndef PATH_DIST_COST_FUNCTION_H_
#define PATH_DIST_COST_FUNCTION_H_

#include <plp_basic_cost_functions/map_grid_cost_function.h>

namespace plp_basic_cost_functions {

class PathDistCostFunction: public MapGridCostFunction {
public:
  bool prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec);
};

} /* namespace plp_basic_cost_functions */
#endif /* PATH_DIST_COST_FUNCTION_H_ */
