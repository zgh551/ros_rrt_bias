#ifndef _PLANNER_H_
#define _PLANNER_H_

/*
 * @brief The ROS related header file
 */
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
// the msg of start and goal pose
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
// the marker in rviz
#include <visualization_msgs/Marker.h>
// the pose array
#include <geometry_msgs/PoseArray.h>
// the path in rviz
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
/*
 * @brief OMPL lib
 */
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/geometric/SimpleSetup.h>

/*
 * @brief The State space
 */
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>


/*
 * @brief the namespace
 */
namespace ob = ompl::base;
namespace og = ompl::geometric;

/*
 * @brief the micro define
 */
#define BOUNDARY    (10)

typedef struct _position
{
    double x;
    double y;
    double yaw;
}Position;

namespace RRT_planner
{
    class Planner
    {
        public:
        /*
         * @brief Construct function
         */
        Planner(void);
        ~Planner(void);

        /*
         * @brief Init the planner,such as,the space information, the obstacle
         * checker and the path cost optimization
         */
        void Init(void);

        /*
         * @brief The solve of planner
         */
        void solve(const double time);

        /*
         * @brief The callback function map receive
         */
        void MapCallback(const nav_msgs::OccupancyGrid::Ptr map);

        /*
         * @brief The callback function of start pose
         */
        void StartPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &pose);

        /*
         * @brief The callback function of goal pose
         */
        void GoalPoseCallback(const geometry_msgs::PoseStamped &pose);
        private:
        /*
         * @brief the node handle  
         */
        ros::NodeHandle n;

        /*
         * @brief the map subscriber
         */
        ros::Subscriber map_sub;

        /*
         * @brief the subscriber for receive the start update
         */
        ros::Subscriber start_pose_sub;
        /*
         * @brief the subscriber for receive the goal update
         */
        ros::Subscriber goal_pose_sub;

        /*
         * @brief the line marker 
         */
        ros::Publisher line_marker_pub;

        /*
         * @brief the pose array publisher
         */
        ros::Publisher pose_array_pub;

        /*
         * @brief the path to publisher
         */
        ros::Publisher path_state_pub;

        /*
         * @brief the start position line strip
         */
        visualization_msgs::Marker _start_pose_line_strip;

        /*
         * @brief the goal position line strip
         */
        visualization_msgs::Marker _goal_pose_line_strip;

        /*
         * @brief the rrt sample point show in rviz
         */
        geometry_msgs::PoseArray _sample_point; 

        /*
         * @brief the plan path by the planner
         */
        nav_msgs::Path _plan_path;

        nav_msgs::OccupancyGrid _cost_map[36];
        /*
         * @brief The Space information
         */
        ob::SpaceInformationPtr _si;

        /*
         * @brief The state space
         */
        ob::StateSpacePtr _state_space;
        /*
         * @brief The simple setup pointer
         */
        og::SimpleSetupPtr _ss;

        /*
         * @brief The height of map
         */
        double _map_height;

        /*
         * @brief The width of map
         */
        double _map_width;        

        /*
         * @brief start position
         */
        Position _start_position;
        bool     _start_valid;

        /*
         * @brief goal position
         */
        Position _goal_position;
        bool     _goal_valid;

    };
}

#endif
