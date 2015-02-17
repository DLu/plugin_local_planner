#ifndef OFFSET_GRID_COST_FUNCTION_H_
#define OFFSET_GRID_COST_FUNCTION_H_

#include <plp_basic_cost_functions/map_grid_cost_function.h>

namespace plp_basic_cost_functions {

class OffsetGridCostFunction: public MapGridCostFunction {
public:

  virtual bool prepare(tf::Stamped<tf::Pose> global_pose,
		       tf::Stamped<tf::Pose> global_vel,
		       std::vector<geometry_msgs::Point> footprint_spec) = 0;
  virtual double scoreCell(double px, double py, double pth);
protected:
    double xshift_, yshift_;
    double shift_d_;
    bool quit_within_radius_;
};

} /* namespace plp_basic_cost_functions */
#endif /* OFFSET_GRID_COST_FUNCTION_H_ */
