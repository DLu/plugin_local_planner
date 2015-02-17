#include <plp_basic_cost_functions/offset_grid_cost_function.h>

using plugin_local_planner::Trajectory;

namespace plp_basic_cost_functions {

double OffsetGridCostFunction::scoreCell(double px, double py, double pth)
{
    if(quit_within_radius_){
        double d = sqrt(pow(px-goal_x_,2)+pow(py-goal_y_,2));
        if(d<shift_d_){
            return 0.0;
        }
    }
    

    // translate point forward if specified
    if (xshift_ != 0.0) {
      px = px + xshift_ * cos(pth);
      py = py + xshift_ * sin(pth);
    }
    // translate point sideways if specified
    if (yshift_ != 0.0) {
      px = px + yshift_ * cos(pth + M_PI_2);
      py = py + yshift_ * sin(pth + M_PI_2);
    }

    double g = MapGridCostFunction::scoreCell(px, py, pth);  
    // ROS_INFO("\t\t%.2f %.2f %.2f | %.2f", px, py, pth, g);
    return g;
}
} /* namespace plp_basic_cost_functions */

 
