/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <plugin_local_planner/plugin_planner.h>
#include <cmath>

//for computing path distance
#include <queue>

#include <angles/angles.h>

#include <ros/ros.h>

class XmlArray :public XmlRpc::XmlRpcValue
{
public:

  void setArray(std::vector<XmlRpc::XmlRpcValue>* a)
  {
    _type = TypeArray;
    _value.asArray = new std::vector<XmlRpc::XmlRpcValue>(*a);
  }
};

namespace plugin_local_planner {
void move_parameter(ros::NodeHandle& nh, std::string old_name, 
                    std::string name, double default_value, bool should_delete=true)
{
    if (nh.hasParam(name)){
        if(should_delete)
            nh.deleteParam(old_name);
        return;
    }



    XmlRpc::XmlRpcValue value;
    if (nh.hasParam(old_name))
    {
        nh.getParam(old_name, value);
        if(should_delete) nh.deleteParam(old_name);
    }
    else
        value = default_value;

    nh.setParam(name, value);
}


  void PluginPlanner::reconfigure(PluginPlannerConfig &config)
  {

    boost::mutex::scoped_lock l(configuration_mutex_);

    generator_.setParameters(
        config.sim_time,
        config.sim_granularity,
        config.angular_sim_granularity,
        config.use_dwa,
        sim_period_);

    stop_time_buffer_ = config.stop_time_buffer;

    int vx_samp, vy_samp, vth_samp;
    vx_samp = config.vx_samples;
    vy_samp = config.vy_samples;
    vth_samp = config.vth_samples;
 
    if (vx_samp <= 0) {
      ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
      vx_samp = 1;
      config.vx_samples = vx_samp;
    }
 
    if (vy_samp <= 0) {
      ROS_WARN("You've specified that you don't want any samples in the y dimension. We'll at least assume that you want to sample one value... so we're going to set vy_samples to 1 instead");
      vy_samp = 1;
      config.vy_samples = vy_samp;
    }
 
    if (vth_samp <= 0) {
      ROS_WARN("You've specified that you don't want any samples in the th dimension. We'll at least assume that you want to sample one value... so we're going to set vth_samples to 1 instead");
      vth_samp = 1;
      config.vth_samples = vth_samp;
    }
 
    vsamples_[0] = vx_samp;
    vsamples_[1] = vy_samp;
    vsamples_[2] = vth_samp;
 

  }

  PluginPlanner::PluginPlanner(std::string name, plugin_local_planner::LocalPlannerUtil *planner_util) :
      planner_util_(planner_util), plugin_loader_("plugin_local_planner", "plugin_local_planner::TrajectoryCostFunction")
  {
    ros::NodeHandle private_nh("~/" + name);

    //Assuming this planner is being run within the navigation stack, we can
    //just do an upward search for the frequency at which its being run. This
    //also allows the frequency to be overwritten locally.
    std::string controller_frequency_param_name;
    if(!private_nh.searchParam("controller_frequency", controller_frequency_param_name)) {
      sim_period_ = 0.05;
    } else {
      double controller_frequency = 0;
      private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
      if(controller_frequency > 0) {
        sim_period_ = 1.0 / controller_frequency;
      } else {
        ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
        sim_period_ = 0.05;
      }
    }
    ROS_INFO("Sim period is set to %.2f", sim_period_);

    reset();

    std::string frame_id;
    private_nh.param("global_frame_id", frame_id, std::string("odom"));

    if (!private_nh.hasParam("critics"))
    {
        resetOldParameters(private_nh);
    }

    if (private_nh.hasParam("critics"))
    {
        XmlRpc::XmlRpcValue my_list;
        private_nh.getParam("critics", my_list);
        for (int32_t i = 0; i < my_list.size(); ++i)
        {
          std::string pname, type;
          double scale = 0.0;

          if(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
              pname = static_cast<std::string>(my_list[i]["name"]);
              type = static_cast<std::string>(my_list[i]["type"]);
              scale = my_list[i]["scale"];
          }else{
              pname = static_cast<std::string>(my_list[i]);
              type = pname;
          }

          if(type.find("CostFunction") == std::string::npos)
          {
              type = type + "CostFunction";
          }

          if(type.find("::") == std::string::npos)
          {
              if(plugin_loader_.isClassAvailable("plp_basic_cost_functions::"+type))
                  type = "plp_basic_cost_functions::" + type;              
              else if(plugin_loader_.isClassAvailable("dwa_plugins::"+type))
                  type = "dwa_plugins::"+type;
          }

          ROS_INFO("Using critic \"%s\"", pname.c_str());

          CostFunctionPointer plugin = plugin_loader_.createInstance(type);
          critics_.push_back(plugin);
          plugin->initialize(name + "/" + pname, planner_util); 
          if(scale>0.0)
            plugin->setScale(scale);
          
          ROS_INFO("     (scale: %f)", plugin->getScale());
        }
    }
    scale_manager_.initialize(&critics_);

    // trajectory generators
    std::vector<plugin_local_planner::TrajectorySampleGenerator*> generator_list;
    generator_list.push_back(&generator_);

    bool debug_paths;
    private_nh.param("debug_paths", debug_paths, false);
    scored_sampling_planner_ = SimpleScoredSamplingPlanner(generator_list, critics_, -1, debug_paths);
  }

  void PluginPlanner::resetOldParameters(ros::NodeHandle& nh)
  {
    ROS_INFO("Loading from pre-hydro parameter style");
    std::vector < XmlRpc::XmlRpcValue > plugins;
    plugins.resize(6);
    plugins[0] = "Oscillation"; // discards oscillating motions (assisgns cost -1)
    plugins[1] = "Obstacle";    // discards trajectories that move into obstacles
    plugins[2] = "GoalAlign";   // prefers trajectories that make the nose go towards (local) nose goal
    plugins[3] = "PathAlign";   // prefers trajectories that keep the robot nose on nose path
    plugins[4] = "PathDist";    // prefers trajectories on global path
    plugins[5] = "GoalDist";    // prefers trajectories that go towards (local) goal, based on wave propagation

    XmlArray critics;
    critics.setArray(&plugins);
    nh.setParam("critics", critics);

    move_parameter(nh, "path_distance_bias", "PathAlign/scale", 32.0, false);
    move_parameter(nh, "goal_distance_bias", "GoalAlign/scale", 24.0, false);
    move_parameter(nh, "path_distance_bias", "PathDist/scale", 32.0);
    move_parameter(nh, "goal_distance_bias", "GoalDist/scale", 24.0);
    move_parameter(nh, "occdist_scale",      "Obstacle/scale", 0.01);

    move_parameter(nh, "max_scaling_factor", "Obstacle/max_scaling_factor", 0.2);
    move_parameter(nh, "scaling_speed",      "Obstacle/scaling_speed", 0.25);

}


  // used for visualization only, total_costs are not really total costs
  bool PluginPlanner::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {
    total_cost = 0.0;
    COST_ITERATOR(critic, critics_){
        CostFunctionPointer cp = *critic;
        std::string name = cp->getName();
        if(name.find("Obstacle")!=std::string::npos){
            occ_cost = cp->getCost(cx, cy);
            total_cost += occ_cost * cp->getScale();
        }else if(name.find("PathDist")!=std::string::npos){
            path_cost = cp->getCost(cx, cy);
	            total_cost += path_cost * cp->getScale();
        }else if(name.find("GoalDist")!=std::string::npos){
            goal_cost = cp->getCost(cx, cy);
            total_cost += goal_cost * cp->getScale();
        }
    }

    return occ_cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }

  bool PluginPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    reset();
    return planner_util_->setPlan(orig_global_plan);
  }

  /**
   * This function is used when other strategies are to be applied,
   * but the cost functions for obstacles are to be reused.
   */
  bool PluginPlanner::checkTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      Eigen::Vector3f vel_samples){
      
    double cost = scoreTrajectory(pos, vel, vel_samples);
    //if the trajectory is a legal one... the check passes
    if(cost >= 0) {
      return true;
    }
    ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

    //otherwise the check fails
    return false;
  }
  
  
  double PluginPlanner::scoreTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      Eigen::Vector3f vel_samples){    
    reset();
    plugin_local_planner::Trajectory traj;
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation));
    plugin_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
    generator_.initialise(pos,
        vel,
        goal,
        &limits,
        vsamples_);
        
    if(generator_.generateTrajectory(pos, vel, vel_samples, traj))
        return scored_sampling_planner_.scoreTrajectory(traj, -1);
    else
        return -1.0;
  }


  void PluginPlanner::updatePlanAndLocalCosts(
      tf::Stamped<tf::Pose> global_pose,
      const std::vector<geometry_msgs::PoseStamped>& new_plan) {
    global_plan_.resize(new_plan.size());
    for (unsigned int i = 0; i < new_plan.size(); ++i) {
      global_plan_[i] = new_plan[i];
    }
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();

    double gx = goal_pose.pose.position.x, gy = goal_pose.pose.position.y;

    COST_ITERATOR(critic, critics_){
        (*critic)->setGlobalPlan(global_plan_, gx, gy);
    }
  }


  /*
   * given the current state of the robot, find a good trajectory
   */
  plugin_local_planner::Trajectory PluginPlanner::findBestPath(
      tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      tf::Stamped<tf::Pose>& drive_velocities,
      std::vector<geometry_msgs::Point> footprint_spec) {

    //make sure that our configuration doesn't change mid-run
    boost::mutex::scoped_lock l(configuration_mutex_);

    Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
    Eigen::Vector3f vel(global_vel.getOrigin().getX(), global_vel.getOrigin().getY(), tf::getYaw(global_vel.getRotation()));
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation));
    plugin_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

    // prepare cost functions and generators for this run
    generator_.initialise(pos,
        vel,
        goal,
        &limits,
        vsamples_);

    result_traj_.cost_ = -7;
    // find best trajectory by sampling and scoring the samples
    std::vector<plugin_local_planner::Trajectory> all_explored;

	prepare(global_pose, global_vel, footprint_spec);
    scored_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);

    // debrief stateful scoring functions
    COST_ITERATOR(critic, critics_){
        (*critic)->debrief(result_traj_);
    }

    //if we don't have a legal trajectory, we'll just command zero
    if (result_traj_.cost_ < 0) {
      drive_velocities.setIdentity();
    } else {
      tf::Vector3 start(result_traj_.xv_, result_traj_.yv_, 0);
      drive_velocities.setOrigin(start);
      tf::Matrix3x3 matrix;
      matrix.setRotation(tf::createQuaternionFromYaw(result_traj_.thetav_));
      drive_velocities.setBasis(matrix);
    }

    return result_traj_;
  }
  
  
  void PluginPlanner::prepare(
          tf::Stamped<tf::Pose> global_pose,
          tf::Stamped<tf::Pose> global_vel,
          std::vector<geometry_msgs::Point> footprint_spec){
  
    COST_ITERATOR(critic, critics_){
      if ((*critic)->prepare(global_pose, global_vel, footprint_spec) == false) {
        ROS_WARN("A scoring function failed to prepare");
      }
    }
  }
  
};
