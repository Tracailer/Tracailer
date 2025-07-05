#include "planner/planner.h"

namespace trailer_planner
{
    void Planner::init(ros::NodeHandle& nh)
    {
        trailer.reset(new Trailer);
        grid_map.reset(new GridMap);
        
        trailer->init(nh);
        grid_map->init(nh);

        hybrid_astar.init(nh);
        hybrid_astar.setTrailerEnv(trailer, grid_map);
        arc_opt.init(nh);
        arc_opt.setTrailerEnv(trailer, grid_map);

        front_pub = nh.advertise<visualization_msgs::MarkerArray>("/front_path", 1);
        end_pub = nh.advertise<visualization_msgs::MarkerArray>("/end_path", 1);
        arc_traj_pub = nh.advertise<planner::ArcTrailerTraj>("/arc_trailer_traj", 1);

        wps_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, &Planner::rcvWpsCallBack, this);
        odom_sub = nh.subscribe("odom", 1, &Planner::rcvOdomCallBack, this);
    
        start_pos.resize(TRAILER_NUM+3);
        end_pos.resize(TRAILER_NUM+3);
        front_path.clear();

        return;
    }

    void Planner::rcvOdomCallBack(planner::TrailerStatePtr msg)
    {
        has_odom = true;
        Eigen::VectorXd s;
        s.resize(TRAILER_NUM+3);
        s[0] = msg->odoms[0].pose.pose.position.x;
        s[1] = msg->odoms[0].pose.pose.position.y;
        s[2] = 2.0 * atan2(msg->odoms[0].pose.pose.orientation.z, msg->odoms[0].pose.pose.orientation.w);
        for (int i=1; i<=TRAILER_NUM; i++)
            s[2+i] = 2.0 * atan2(msg->odoms[i].pose.pose.orientation.z, msg->odoms[i].pose.pose.orientation.w);
        odom_pos = s;
        odom_vw[0] = msg->odoms[0].twist.twist.linear.x;
        odom_vw[1] = msg->odoms[0].twist.twist.angular.z;
        return;
    }

    void Planner::rcvWpsCallBack(const geometry_msgs::PoseWithCovarianceStamped msg)
    {
        static size_t cnt = 0;     

        if (cnt > 0)
            return; 

        Eigen::VectorXd far;
        far.resize(TRAILER_NUM+3);
        far.fill(1000);

        if (in_plan)
            return;

        Eigen::Vector3d wps(22.6, 5.86, 0.0);
        
        if ((wps.head(2)-odom_pos.head(2)).norm() < 0.5)
            return;
        
        if (!has_odom)
        {
            PRINT_RED("[Planner] No odom received, cannot plan.");
            return;
        }
        start_pos = odom_pos;
        if (cnt == 0)
        {
            cnt ++;
            end_pos = wps;
            trailer->setShowTerminal(wps);
            PRINT_YELLOW("[Planner] Get target pose, begin planning.");
            in_plan = true;
            if (plan(start_pos, end_pos))
            {
                PRINT_GREEN("[Planner] Planning done.");
            }
            else
            {
                PRINT_RED("[Planner] Planning failed, check bugs!");
                front_path.clear();
            }
            in_plan = false;
        }
        
        return;
    }

    bool Planner::plan(Eigen::VectorXd start, Eigen::VectorXd end)
    {
        assert(end.size() == 3);

        Eigen::VectorXd end_full;
        trailer->setStateFromBox(end, end_full);

        ros::Time start_time = ros::Time::now();
        front_path = hybrid_astar.pureAstarPlan(start, end_full);
        PRINT_GREEN("[Planner] front end time: "<<(ros::Time::now() - start_time).toSec()<<"s");

        if (front_path.empty())
            return false;

        vis_front();

        // arc opt
        if (!arc_opt.optimizeTraj(front_path))
            return false;

        arc_traj = arc_opt.getTraj();

        vis_end();
        pub_end();

        return true;
    }

    void Planner::vis_front()
    {
        if (front_path.empty())
            return;

        visualization_msgs::MarkerArray front_msg;
        std::vector<Eigen::VectorXd> se2_path; 
        for (size_t i=0; i<front_path.size(); i++)
        {
            Eigen::VectorXd se2_state;
            trailer->gainSE2State(front_path[i], se2_state);
            se2_path.push_back(se2_state);
        }
        for (size_t i=0; i<TRAILER_NUM+1; i++)
        {
            visualization_msgs::Marker sphere, line_strip;
            sphere.header.frame_id = line_strip.header.frame_id = "world";
            sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
            sphere.type = visualization_msgs::Marker::SPHERE_LIST;
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;
            sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
            sphere.id = i;
            line_strip.id = i + 1000;

            sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
            sphere.color.r = line_strip.color.r = 0.8;
            sphere.color.g = line_strip.color.g = 0.0;
            sphere.color.b = line_strip.color.b = 0.0;
            sphere.color.a = line_strip.color.a = 1;
            sphere.scale.x = 0.1;
            sphere.scale.y = 0.1;
            sphere.scale.z = 0.1;
            line_strip.scale.x = 0.1 / 2;
            geometry_msgs::Point pt;
            
            for (auto p:se2_path)
            {
                pt.x = p[3*i];
                pt.y = p[3*i+1];
                pt.z = 0.0;
                line_strip.points.push_back(pt);
                sphere.points.push_back(pt);
            }
            front_msg.markers.push_back(line_strip);
            front_msg.markers.push_back(sphere);
        }
        front_pub.publish(front_msg);
    }

    void Planner::vis_end()
    {
        double ttime_duration;
        ttime_duration = arc_traj.getTotalDuration();
        if (ttime_duration<1e-3)
            return;

        visualization_msgs::MarkerArray end_msg;
        std::vector<Eigen::VectorXd> se2_path;

        for (double i = 0; i < ttime_duration + 0.01; i+=0.01)
        {
            double t = i;
            if (i > ttime_duration)
                t = ttime_duration;
            
            Eigen::VectorXd se2_state;
            Eigen::VectorXd state;
            state = arc_traj.getState(t);
            trailer->gainSE2State(state, se2_state);
            se2_path.push_back(se2_state);
        }

        for (size_t i=0; i<TRAILER_NUM+1; i++)
        {
            double scale = 0.2;
            visualization_msgs::Marker line_strip;
            line_strip.header.frame_id = "world";
            line_strip.header.stamp = ros::Time::now();
            line_strip.type = visualization_msgs::Marker::LINE_STRIP;
            line_strip.action = visualization_msgs::Marker::ADD;
            line_strip.id = i + 1000;

            line_strip.pose.orientation.w = 1.0;
            line_strip.color.r = trailer->array_msg.markers[i].color.r;
            line_strip.color.g = trailer->array_msg.markers[i].color.g;
            line_strip.color.b = trailer->array_msg.markers[i].color.b;
            line_strip.color.a = 1;
            line_strip.scale.x = scale / 2;
            geometry_msgs::Point pt;
            
            for (auto p:se2_path)
            {
                pt.x = p[3*i];
                pt.y = p[3*i+1];
                pt.z = 0.0;
                line_strip.points.push_back(pt);
            }
            end_msg.markers.push_back(line_strip);
        }
        end_pub.publish(end_msg);
    }

    void Planner::pub_end()
    {
        planner::ArcTrailerTraj traj_msg;
        traj_msg.head.start_time = traj_msg.tails.start_time = ros::Time::now();
        traj_msg.arc.start_time = ros::Time::now();
        Eigen::VectorXd durs = arc_traj.head.getDurations();
        for (int i=0; i<durs.size(); i++)
        {
            traj_msg.head.durations.push_back((float)durs[i]);
            std_msgs::Float32MultiArray coeff_mat;
            for (int j=0; j<2; j++)
            {
                for (int k=0; k<6; k++)
                {
                    coeff_mat.data.push_back((float)(arc_traj.head[i].getCoeffMat()(j, k)));
                }
            }
            traj_msg.head.coeff.push_back(coeff_mat);
        }
        durs = arc_traj.tails.getDurations();
        for (int i=0; i<durs.size(); i++)
        {
            traj_msg.tails.durations.push_back((float)durs[i]);
            std_msgs::Float32MultiArray coeff_mat;
            for (int j=0; j<TRAILER_NUM; j++)
            {
                for (int k=0; k<6; k++)
                {
                    coeff_mat.data.push_back((float)(arc_traj.tails[i].getCoeffMat()(j, k)));
                }
            }
            traj_msg.tails.coeff.push_back(coeff_mat);
        }
        durs = arc_traj.arc.getDurations();
        for (int i=0; i<durs.size(); i++)
        {
            traj_msg.arc.durations.push_back((float)durs[i]);
            std_msgs::Float32MultiArray coeff_mat;
            for (int j=0; j<1; j++)
            {
                for (int k=0; k<6; k++)
                {
                    coeff_mat.data.push_back((float)(arc_traj.arc[i].getCoeffMat()(j, k)));
                }
            }
            traj_msg.arc.coeff.push_back(coeff_mat);
        }

        arc_traj_pub.publish(traj_msg);
        return;
    }

}