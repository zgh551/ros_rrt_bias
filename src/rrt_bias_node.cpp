// ros lib
#include "ros/ros.h"
#include "../include/planner.h"


int main(int argc, char** argv)
{ 
    ros::init(argc, argv, "ompl_rrt_bias");

    ros::NodeHandle n;
    
    ros::Rate r(20);


    RRT_planner::Planner m_planner;
    m_planner.Init();

    while(ros::ok())
    {
        m_planner.solve(10);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}

