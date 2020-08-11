#ifndef _VISUALIZE_H_
#define _VISUALIZE_H_

/*
 * @brief OMPL
 */
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

/*
 * @brief OMPL lib
 */
#include <ompl/base/State.h>


namespace ob = ompl::base;

namespace RRT_planner
{
    class Visualize
    {
        public:
            Visualize();
            ~Visualize() = default;

            void clear(void);

            /*
             * @brief publish a single point to rviz visualize
             * @param state: the state of point
             * @return None
             */
            void publishPoint(ob::State *state);

            /*
             * @brief publish an array points
             */
            void publishPoints(ob::State *state);
        private:
            /*
             * @brief The node handle
             */
            ros::NodeHandle _n;

            /*
             * @brief The publisher
             */
            // publisher for a single point
            ros::Publisher _pub_point;
            
            // Publisher for an array of points
            ros::Publisher _pub_points;

            /*
             * @brief pose array
             */
            geometry_msgs::PoseArray _pose_points;
    };
}
#endif
