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
#ifndef PLUGIN_LOCAL_PLANNER_PLUGIN_PLANNER_H_
#define PLUGIN_LOCAL_PLANNER_PLUGIN_PLANNER_H_

#include <vector>
#include <Eigen/Core>


#include <plugin_local_planner/PluginPlannerConfig.h>

//for obstacle data access
#include <costmap_2d/costmap_2d.h>

#include <plugin_local_planner/trajectory.h>
#include <plugin_local_planner/local_planner_limits.h>
#include <plugin_local_planner/local_planner_util.h>
#include <plugin_local_planner/simple_trajectory_generator.h>

#include <plugin_local_planner/trajectory_cost_function.h>
#include <plugin_local_planner/simple_scored_sampling_planner.h>
#include <plugin_local_planner/scale_manager.h>

#include <nav_msgs/Path.h>
#include <pluginlib/class_loader.h>

namespace plugin_local_planner {
  /**
   * @class PluginPlanner
   * @brief A class implementing a local planner using the Dynamic Window Approach
   */
  class PluginPlanner {
    public:
      /**
       * @brief  Constructor for the planner
       * @param name The name of the planner 
       * @param costmap_ros A pointer to the costmap instance the planner should use
       * @param global_frame the frame id of the tf frame to use
       */
      PluginPlanner(std::string name, plugin_local_planner::LocalPlannerUtil *planner_util);

      /**
       * @brief  Destructor for the planner
       */
      ~PluginPlanner() {}

      /**
       * @brief Reconfigures the trajectory planner
       */
      void reconfigure(PluginPlannerConfig &cfg);

      /**
       * @brief  Check if a trajectory is legal for a position/velocity pair
       * @param pos The robot's position
       * @param vel The robot's velocity
       * @param vel_samples The desired velocity
       * @return True if the trajectory is valid, false otherwise
       */
      bool checkTrajectory(
          const Eigen::Vector3f pos,
          const Eigen::Vector3f vel,
          const Eigen::Vector3f vel_samples);
          
      /**
       * @brief  Score a trajectory for a position/velocity pair
       * @param pos The robot's position
       * @param vel The robot's velocity
       * @param vel_samples The desired velocity
       * @return True if the trajectory is valid, false otherwise
       */
      double scoreTrajectory(
          const Eigen::Vector3f pos,
          const Eigen::Vector3f vel,
          const Eigen::Vector3f vel_samples);

      /**
       * @brief Given the current position and velocity of the robot, find the best trajectory to execute
       * @param global_pose The current position of the robot 
       * @param global_vel The current velocity of the robot 
       * @param drive_velocities The velocities to send to the robot base
       * @return The highest scoring trajectory. A cost >= 0 means the trajectory is legal to execute.
       */
      plugin_local_planner::Trajectory findBestPath(
          tf::Stamped<tf::Pose> global_pose,
          tf::Stamped<tf::Pose> global_vel,
          tf::Stamped<tf::Pose>& drive_velocities,
          std::vector<geometry_msgs::Point> footprint_spec);

      /**
       * @brief Pass the current position and velocity of the robot to the trajectory cost functions
       * @param global_pose The current position of the robot 
       * @param global_vel The current velocity of the robot 
       * @param drive_velocities The velocities to send to the robot base
       * @return The highest scoring trajectory. A cost >= 0 means the trajectory is legal to execute.
       */          
      void prepare(
          tf::Stamped<tf::Pose> global_pose,
          tf::Stamped<tf::Pose> global_vel,
          std::vector<geometry_msgs::Point> footprint_spec);

      /**
       * @brief  Take in a new global plan for the local planner to follow, and adjust local costmaps
       * @param  new_plan The new global plan
       */
      void updatePlanAndLocalCosts(tf::Stamped<tf::Pose> global_pose,
          const std::vector<geometry_msgs::PoseStamped>& new_plan);

      /**
       * @brief Get the period at which the local planner is expected to run
       * @return The simulation period
       */
      double getSimPeriod() { return sim_period_; }

      /**
       * @brief Compute the components and total cost for a map grid cell
       * @param cx The x coordinate of the cell in the map grid
       * @param cy The y coordinate of the cell in the map grid
       * @param path_cost Will be set to the path distance component of the cost function
       * @param goal_cost Will be set to the goal distance component of the cost function
       * @param occ_cost Will be set to the costmap value of the cell
       * @param total_cost Will be set to the value of the overall cost function, taking into account the scaling parameters
       * @return True if the cell is traversible and therefore a legal location for the robot to move to
       */
      bool getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost);

      /**
       * sets new plan and resets state
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    private:
      void reset() { 
        COST_ITERATOR(critic, critics_){ (*critic)->reset(); }
      }

      void resetOldParameters(ros::NodeHandle& nh);

      plugin_local_planner::LocalPlannerUtil *planner_util_;

      double stop_time_buffer_; ///< @brief How long before hitting something we're going to enforce that the robot stop
      double pdist_scale_, gdist_scale_, occdist_scale_;
      double porient_scale_, gorient_scale_;
      Eigen::Vector3f vsamples_;

      double sim_period_;///< @brief The number of seconds to use to compute max/min vels for dwa
      plugin_local_planner::Trajectory result_traj_;

      double forward_point_distance_;
      double scaled_path_factor_;

      std::vector<geometry_msgs::PoseStamped> global_plan_;

      boost::mutex configuration_mutex_;
      bool publish_cost_grid_pc_; ///< @brief Whether or not to build and publish a PointCloud
      bool publish_traj_pc_;

      double cheat_factor_;

      // see constructor body for explanations
      plugin_local_planner::SimpleTrajectoryGenerator generator_;

      pluginlib::ClassLoader<TrajectoryCostFunction> plugin_loader_;
      std::vector<CostFunctionPointer > critics_;

      plugin_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;

      plugin_local_planner::ScaleManager scale_manager_;
  };
};
#endif
