// ros lib
#include "ros/ros.h"


int main(int argc, char** argv)
{ 
    ros::init(argc, argv, "ompl_rrt_bias");

    ros::NodeHandle n;
    
    ros::Rate r(20);

    while(ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

