/*
 * @author guohua zhu
 * @time 2020/8/11
 * @brief the rviz visualize 
 */
#include "../include/visualize.h"
/*
 * ROS
 */
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/time.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

/*
 * @brief OMPL lib
 */
#include <ompl/base/State.h>
#include <ompl/base/spaces/SE2StateSpace.h>


RRT_planner::Visualize::Visualize()
{
    _pub_point = _n.advertise<geometry_msgs::PoseStamped>("/visualize_point", 100);
    _pub_points = _n.advertise<geometry_msgs::PoseArray>("/visualize_points", 100);
    
    _pose_points.header.frame_id = "map";        
}


void RRT_planner::Visualize::clear(void)
{
    _pose_points.poses.clear();
}

void RRT_planner::Visualize::publishPoint(ob::State *state)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.header.seq = 0;

    auto *SE2_state = state->as<ob::SE2StateSpace::StateType>();
    
    pose.pose.position.x  = SE2_state->getX();
    pose.pose.position.y  = SE2_state->getY();
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(SE2_state->getYaw());

    _pub_point.publish(pose);
}

void RRT_planner::Visualize::publishPoints(ob::State *state)
{
    geometry_msgs::Pose pose;
    auto *SE2_state = state->as<ob::SE2StateSpace::StateType>();

    pose.position.x = SE2_state->getX();
    pose.position.y = SE2_state->getY();
    pose.orientation = tf::createQuaternionMsgFromYaw(SE2_state->getYaw());

    _pose_points.poses.push_back(pose);
    _pose_points.header.stamp = ros::Time::now();

    _pub_points.publish(_pose_points);
}
