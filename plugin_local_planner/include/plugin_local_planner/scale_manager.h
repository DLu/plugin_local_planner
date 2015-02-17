#ifndef DWA_SCALE_MANAGER_H_
#define DWA_SCALE_MANAGER_H_

#include<dynamic_reconfigure/Reconfigure.h>
#include<dynamic_reconfigure/ConfigDescription.h>
#include<dynamic_reconfigure/Group.h>
#include<dynamic_reconfigure/ParamDescription.h>
#include<dynamic_reconfigure/DoubleParameter.h>
#include<dynamic_reconfigure/Config.h>
#include<plugin_local_planner/trajectory_cost_function.h>

namespace plugin_local_planner {

class ScaleManager {
public:
    ScaleManager() { }
    void initialize(std::vector<CostFunctionPointer >* critics_);
    bool set_params(dynamic_reconfigure::Reconfigure::Request &req,
                    dynamic_reconfigure::Reconfigure::Response &res);

private:
    std::vector<CostFunctionPointer >* critics_;
    dynamic_reconfigure::ConfigDescription description_;
    ros::Publisher description_pub_;
    ros::ServiceServer set_params_srv_;
};
}


#endif /* DWA_SCALE_MANAGER_H_ */
