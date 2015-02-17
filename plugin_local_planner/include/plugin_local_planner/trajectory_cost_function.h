#ifndef DWA_TRAJECTORYCOSTFUNCTION_H_
#define DWA_TRAJECTORYCOSTFUNCTION_H_

#define COST_ITERATOR(X, Y)  for(std::vector<CostFunctionPointer >::iterator X = Y.begin(); X != Y.end(); ++X)
#define COST_ITERATOR_P(X, Y)  for(std::vector<CostFunctionPointer >::iterator X = Y->begin(); X != Y->end(); ++X)

#include <plugin_local_planner/trajectory.h>
#include <plugin_local_planner/local_planner_util.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>

namespace plugin_local_planner {

/**
 * @class TrajectoryCostFunction
 * @brief Provides an interface for critics of trajectories
 * During each sampling run, a batch of many trajectories will be scored using such a cost function.
 * The prepare method is called before each batch run, and then for each
 * trajectory of the sampling set, score_trajectory may be called.
 */
class TrajectoryCostFunction {
public:

  virtual void initialize(std::string name, plugin_local_planner::LocalPlannerUtil *planner_util) {
    name_ = name;
    planner_util_ = planner_util;
    costmap_ = planner_util->getCostmap();

    ros::NodeHandle nh("~/" + name_);
    nh.param("scale", scale_, 1.0);

  }

  /**
   * General updating of context values if required.
   * Subclasses may overwrite. Return false in case there is any error.
   */
  virtual bool prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec) = 0;

  virtual void reset() {}

  /**
   * Set Global Plan with plan transformed and cropped into local costmap frame
   */
  virtual void setGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan, double goal_x, double goal_y) {}

  /**
   * return a score for trajectory traj
   */
  virtual double scoreTrajectory(plugin_local_planner::Trajectory &traj) = 0;

  virtual void debrief(plugin_local_planner::Trajectory &result) {}

  double getScale() {
    return scale_;
  }

  void setScale(double scale) {
    scale_ = scale;
  }

  std::string getName() {
    return name_;
  }

  /* Only returns a value for grid based functions */
  virtual float getCost(unsigned int cx, unsigned int cy){ return 0.0; }

  virtual ~TrajectoryCostFunction() {}

protected:
  std::string name_;
  plugin_local_planner::LocalPlannerUtil *planner_util_;
  costmap_2d::Costmap2D* costmap_; 
  double scale_;
};

}


typedef boost::shared_ptr<plugin_local_planner::TrajectoryCostFunction> CostFunctionPointer;

#endif /* TRAJECTORYCOSTFUNCTION_H_ */
