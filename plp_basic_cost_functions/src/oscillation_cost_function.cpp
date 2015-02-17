/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 * Author: TKruse
 *********************************************************************/

#include <plp_basic_cost_functions/oscillation_cost_function.h>

#include <cmath>

PLUGINLIB_EXPORT_CLASS(plp_basic_cost_functions::OscillationCostFunction, plugin_local_planner::TrajectoryCostFunction)

using plugin_local_planner::Trajectory;

namespace plp_basic_cost_functions {

OscillationCostFunction::~OscillationCostFunction() {
  prev_stationary_pos_ = Eigen::Vector3f::Zero();
}

void OscillationCostFunction::initialize(std::string name, plugin_local_planner::LocalPlannerUtil *planner_util)
{
    plugin_local_planner::TrajectoryCostFunction::initialize(name, planner_util);

    oscillation_reset_dist_ = .05; // TODO: Load this dynamically
    oscillation_reset_angle_ = .2; // TODO: Load this dynamically
}

bool OscillationCostFunction::prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec){
    pos_[0] = global_pose.getOrigin().getX();
    pos_[1] = global_pose.getOrigin().getY();
    pos_[2] = tf::getYaw(global_pose.getRotation());
    return true;
}

void OscillationCostFunction::debrief(plugin_local_planner::Trajectory* traj) {
  //Eigen::Vector3f pos;

  double min_vel_trans = planner_util_->getCurrentLimits().min_trans_vel;
  if (traj->cost_ >= 0) {
    if (setOscillationFlags(traj, min_vel_trans)) {
      prev_stationary_pos_ = pos_;
    }
    //if we've got restrictions... check if we can reset any oscillation flags
    if(forward_pos_only_ || forward_neg_only_
        || strafe_pos_only_ || strafe_neg_only_
        || rot_pos_only_ || rot_neg_only_){
      resetOscillationFlagsIfPossible(pos_, prev_stationary_pos_);
    }
  }
}

void OscillationCostFunction::resetOscillationFlagsIfPossible(const Eigen::Vector3f& pos, const Eigen::Vector3f& prev) {
  double x_diff = pos[0] - prev[0];
  double y_diff = pos[1] - prev[1];
  double sq_dist = x_diff * x_diff + y_diff * y_diff;

  double th_diff = pos[2] - prev[2];

  //if we've moved far enough... we can reset our flags
  if (sq_dist > oscillation_reset_dist_ * oscillation_reset_dist_ ||
      fabs(th_diff) > oscillation_reset_angle_) {
    reset();
  }
}

void OscillationCostFunction::reset() {
  strafe_pos_only_ = false;
  strafe_neg_only_ = false;
  strafing_pos_ = false;
  strafing_neg_ = false;

  rot_pos_only_ = false;
  rot_neg_only_ = false;
  rotating_pos_ = false;
  rotating_neg_ = false;

  forward_pos_only_ = false;
  forward_neg_only_ = false;
  forward_pos_ = false;
  forward_neg_ = false;
}

bool OscillationCostFunction::setOscillationFlags(plugin_local_planner::Trajectory* t, double min_vel_trans) {
  bool flag_set = false;
  //set oscillation flags for moving forward and backward
  if (t->xv_ < 0.0) {
    if (forward_pos_) {
      forward_neg_only_ = true;
      flag_set = true;
    }
    forward_pos_ = false;
    forward_neg_ = true;
  }
  if (t->xv_ > 0.0) {
    if (forward_neg_) {
      forward_pos_only_ = true;
      flag_set = true;
    }
    forward_neg_ = false;
    forward_pos_ = true;
  }

  //we'll only set flags for strafing and rotating when we're not moving forward at all
  if (fabs(t->xv_) <= min_vel_trans) {
    //check negative strafe
    if (t->yv_ < 0) {
      if (strafing_pos_) {
        strafe_neg_only_ = true;
        flag_set = true;
      }
      strafing_pos_ = false;
      strafing_neg_ = true;
    }

    //check positive strafe
    if (t->yv_ > 0) {
      if (strafing_neg_) {
        strafe_pos_only_ = true;
        flag_set = true;
      }
      strafing_neg_ = false;
      strafing_pos_ = true;
    }

    //check negative rotation
    if (t->thetav_ < 0) {
      if (rotating_pos_) {
        rot_neg_only_ = true;
        flag_set = true;
      }
      rotating_pos_ = false;
      rotating_neg_ = true;
    }

    //check positive rotation
    if (t->thetav_ > 0) {
      if (rotating_neg_) {
        rot_pos_only_ = true;
        flag_set = true;
      }
      rotating_neg_ = false;
      rotating_pos_ = true;
    }
  }
  return flag_set;
}

double OscillationCostFunction::scoreTrajectory(Trajectory &traj) {
  if ((forward_pos_only_ && traj.xv_ < 0.0) ||
      (forward_neg_only_ && traj.xv_ > 0.0) ||
      (strafe_pos_only_  && traj.yv_ < 0.0) ||
      (strafe_neg_only_  && traj.yv_ > 0.0) ||
      (rot_pos_only_     && traj.thetav_ < 0.0) ||
      (rot_neg_only_     && traj.thetav_ > 0.0)) {
    return -5.0;
  }
  return 0.0;
}

} /* namespace plp_basic_cost_functions */
