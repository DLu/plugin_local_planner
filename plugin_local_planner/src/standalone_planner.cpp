#include <plugin_local_planner/plugin_planner_ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "standalone");
    
    double x = 0, y = 2, yaw=0;
    bool loop = false;
    if(argc>1){
        if(argc>2){
            x = atof(argv[1]);
            y = atof(argv[2]);
            if(argc>3){
                yaw = atof(argv[3]);
            }
        }
        if(strcmp(argv[argc-1], "-l")==0){
            loop = true;
        }
    }

    ROS_INFO("GOAL: %f %f %f", x, y, yaw);

    tf::TransformListener tf;

    plugin_local_planner::PluginPlannerROS planner;
    costmap_2d::Costmap2DROS costmap("local", tf);
    planner.initialize("planner", &tf, &costmap);   

    std::vector<geometry_msgs::PoseStamped> plan;
    int N = 100;
    for(int i=0;i<N;i++){
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "/map";
        pose.pose.orientation.w = 1.0;
        pose.pose.position.x = i * x / N;
        pose.pose.position.y = i * y / N;

        if(i==N-1){
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }

        plan.push_back(pose);
    }
    planner.setPlan(plan);

    geometry_msgs::Twist cmd_vel;
    ros::Rate r(10);

    while(ros::ok()){
    planner.computeVelocityCommands(cmd_vel);
    ROS_INFO("%f %f %f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
    r.sleep();
        if(!loop)
            break;
    }

    return 0;
}
