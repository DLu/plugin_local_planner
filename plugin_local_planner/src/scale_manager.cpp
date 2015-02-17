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
#include <plugin_local_planner/scale_manager.h>

namespace plugin_local_planner {
void ScaleManager::initialize(std::vector<CostFunctionPointer >* critics)
{
    critics_ = critics;
    int N = critics_->size();
    description_.groups.resize(1);
    description_.groups[0].name = "Scales";
    description_.groups[0].parameters.resize(N);
    description_.max.doubles.resize(N);
    description_.min.doubles.resize(N);
    description_.dflt.doubles.resize(N);

    int i = 0;
    COST_ITERATOR_P(critic, critics_){
        std::string name = (*critic)->getName() + "/scale"; 
        description_.groups[0].parameters[i].name = name;
        description_.groups[0].parameters[i].type = "double";
        description_.groups[0].parameters[i].description = "Scale for the " + (*critic)->getName() + " cost function";
        description_.min.doubles[i].name = name;
        description_.min.doubles[i].value = 0.0;
        description_.max.doubles[i].name = name;
        description_.max.doubles[i].value = INFINITY;
        description_.dflt.doubles[i].name = name;
        description_.dflt.doubles[i].value = (*critic)->getScale();
        i++;
    }

    ros::NodeHandle nh("~");
    description_pub_ = nh.advertise<dynamic_reconfigure::ConfigDescription>("scale/parameter_descriptions", 1, true);
    set_params_srv_  = nh.advertiseService("scale/set_parameters", &ScaleManager::set_params, this);

    description_pub_.publish(description_);

}


bool ScaleManager::set_params(dynamic_reconfigure::Reconfigure::Request &req,
                              dynamic_reconfigure::Reconfigure::Response &res)
{
    for(int i=0;i<req.config.doubles.size();i++){
        std::string name = req.config.doubles[i].name;
        double value = req.config.doubles[i].value;

        COST_ITERATOR_P(critic, critics_){
            if(name.find((*critic)->getName()) != std::string::npos){
                (*critic)->setScale(value);
                ROS_INFO("%s set to %f", name.c_str(), value);
                break;
            }
        }
    }
    return true;
}

};
