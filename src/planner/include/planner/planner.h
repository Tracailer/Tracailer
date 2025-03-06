#pragma once

#include <string.h>
#include <iostream>
#include <random>
#include <time.h>
#include <eigen3/Eigen/Eigen>

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include "planner/trailer.hpp"
#include "planner/grid_map.h"
#include "planner/hybrid_astar.h"
#include "planner/arc_opt.h"

#include "planner/ArcTrailerTraj.h"
#include "planner/TrailerState.h"

namespace trailer_planner
{
    class Planner
    {
        private:
            bool has_odom = false;
            bool in_plan = false;

            Eigen::Vector2d odom_vw;
            Eigen::VectorXd odom_pos;
            Eigen::VectorXd start_pos;
            Eigen::VectorXd end_pos;

            // trajs
            std::vector<Eigen::VectorXd> front_path;
            ArcTraj arc_traj;
            
            // members
            Trailer::Ptr trailer;
            GridMap::Ptr grid_map;
            HybridAstar hybrid_astar;
            ArcOpt arc_opt;

            ros::Publisher front_pub, end_pub, arc_traj_pub;
            ros::Subscriber wps_sub, odom_sub;
            
        public:
            void init(ros::NodeHandle& nh);
            void rcvWpsCallBack(const geometry_msgs::PoseWithCovarianceStamped msg);
            void rcvOdomCallBack(planner::TrailerStatePtr msg);
            bool plan(Eigen::VectorXd start, Eigen::VectorXd end);
            void vis_front();
            void vis_end();
            void pub_end();
    };
}
