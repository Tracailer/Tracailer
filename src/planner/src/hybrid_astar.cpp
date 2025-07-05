#include "planner/hybrid_astar.h"

namespace trailer_planner
{
    void HybridAstar::init(ros::NodeHandle& nh)
    {
        nh.param("hybrid_astar/yaw_resolution", yaw_resolution, 3.15);
        nh.param("hybrid_astar/lambda_heu", lambda_heu, 1.0);
        nh.param("hybrid_astar/weight_r2", weight_r2, 1.0);
        nh.param("hybrid_astar/weight_delta", weight_delta, 1.0);
        nh.param("hybrid_astar/weight_v_change", weight_v_change, 0.0);
        nh.param("hybrid_astar/weight_delta_change", weight_delta_change, 0.0);
        nh.param("hybrid_astar/time_interval", time_interval, 0.0);
        nh.param("hybrid_astar/oneshot_range", oneshot_range, 0.0);
        nh.param("hybrid_astar/check_ratio", check_ratio, 1.0);
        nh.param("hybrid_astar/max_vel", max_vel, 1.0);
        nh.param("hybrid_astar/pos_tol", pos_tol, 1.0);
        nh.param("hybrid_astar/theta_tol", theta_tol, 1.0);
        nh.param("hybrid_astar/max_time_consume", max_time_consume, 1.0);
        nh.param("hybrid_astar/in_test", in_test, false);
        nh.param("hybrid_astar/heuristic_type", heuristic_type, 0);

        if (in_test)
        {
            expanded_pub = nh.advertise<sensor_msgs::PointCloud2>("/hybrid_astar/expanded_points", 0);
        }

        yaw_resolution_inv = 1.0 / yaw_resolution;

        return;
    }

    std::vector<Eigen::VectorXd> HybridAstar::pureAstarPlan(const Eigen::VectorXd& start_state, const Eigen::VectorXd& end_state)
    {
        if (!set_done)
        {
            ROS_ERROR("[Hybrid A*] No Setting! Can't begin planning!");
            return front_end_path;
        }

        front_end_path.clear();
        if (!isValid(start_state))
        {
            ROS_ERROR("[Hybrid A*] start is not free!!!");
            return front_end_path;
        }
        if (!isValid(end_state))
        {
            ROS_ERROR("[Hybrid A*] goal is not free!!!");
            return front_end_path;
        }

        front_end_path.push_back(start_state);

        // auto astar_result = grid_map->astarPlan(start_state.head(2), end_state.head(2));
        // auto astar_path = astar_result.first;
        auto astar_path = planAckermann(start_state.head(3), end_state.head(3));
        if (astar_path.empty())
        {
            front_end_path.clear();
            return front_end_path;
        }

        Eigen::VectorXd dtheta;
        dtheta.resize(TRAILER_NUM+1);
        dtheta.setZero();
        Eigen::VectorXd temp_state = start_state;
        for (size_t i=1; i<astar_path.size(); i++)
        {
            Eigen::VectorXd last_state = front_end_path.back();
            double yaw = astar_path[i].z();

            Eigen::VectorXd state;
            double vx = fabs((astar_path[i].x()-astar_path[i-1].x()) / cos(yaw));
            double vy = fabs((astar_path[i].y()-astar_path[i-1].y()) / sin(yaw));
            double v = vx > vy ? vy : vx;

            dtheta(0) = normalizedAngle(yaw - last_state.z());

            // rough set theta
            temp_state.head(3) = astar_path[i].head(3);
            for (size_t j=0; j<TRAILER_NUM; j++)
            {
                double sthetad = sin(last_state(j+2)-last_state(j+3));
                double cthetad = cos(last_state(j+2)-last_state(j+3));
                dtheta(j+1) = (v * sthetad - dtheta(j) * cthetad * trailer->Ltail[j]) / trailer->Lhead[j];
                temp_state(j+3) = normalizedAngle(last_state(j+3) + dtheta(j+1));
                double thetad = temp_state(j+3) - temp_state(j+2);
                if (thetad > M_PI)
                    thetad = thetad - PI_X_2;
                else if (thetad < -M_PI)
                    thetad = thetad + PI_X_2;
                if (thetad > trailer->max_dtheta)
                    temp_state(j+3) = normalizedAngle(temp_state(j+2) + trailer->max_dtheta);
                else if (thetad < -trailer->max_dtheta)
                    temp_state(j+3) = normalizedAngle(temp_state(j+2) - trailer->max_dtheta);
                    
                v = v * cthetad + sthetad * dtheta(j) * trailer->Ltail[j];
            }

            front_end_path.push_back(temp_state);
        }
        // front_end_path.push_back(end_state);

        // PRINT_GREEN("front path:");
        // for (size_t i=0; i<front_end_path.size(); i++)
        // {
        //     PRINT_GREEN(front_end_path[i].transpose());
        // }

        return front_end_path;
    }

    std::vector<Eigen::VectorXd> HybridAstar::planAckermann(const Eigen::Vector3d& start_state, const Eigen::Vector3d& end_state)
    {
        // reset
        int use_node_num = 0;
        int iter_num = 0;
        std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
        open_set.swap(empty_queue);
        std::vector<Eigen::VectorXd> ackermann_path;
        expanded_nodes.clear();
        expanded_points.clear();
        for (int i = 0; i < allocate_num; i++)
        {
            PathNodePtr node = path_node_pool[i];
            node->parent = NULL;
            node->node_state = NOT_EXPAND;
        }

        ros::Time t0 = ros::Time::now();
        PathNodePtr cur_node = path_node_pool[0];
        cur_node->parent = NULL;
        cur_node->state = start_state;
        stateToIndexAckermann(cur_node->state, cur_node->index);
        cur_node->g_score = 0.0;
        cur_node->input = Eigen::Vector2d::Zero();
        cur_node->f_score = lambda_heu * tie_breaker * (cur_node->state.head(2) - end_state.head(2)).norm();
        cur_node->node_state = OPEN;

        Eigen::VectorXi end_index;
        stateToIndexAckermann(end_state, end_index);

        open_set.push(cur_node);
        use_node_num += 1;
        expanded_nodes.insert(cur_node->index, cur_node);
        
        while (!open_set.empty())
        {
            cur_node = open_set.top();

            visExpanded();
            
            if((cur_node->state.head(2) - end_state.head(2)).norm() < oneshot_range)
            {
                // ros::Time t1 = ros::Time::now();
                asignShotTrajAckermann(cur_node->state, end_state);

                if (!shot_path.empty())
                {
                    // std::cout << "[Hybrid A*] one-shot time: " << (ros::Time::now()-t1).toSec()*1000 << " ms"<<std::endl;
                    std::cout << "[Hybrid A*] front once time: " << (ros::Time::now()-t0).toSec()*1000 << " ms"<<std::endl;
                    for (int i=shot_path.size()-1; i>=0; i--)
                        ackermann_path.push_back(shot_path[i]);
                    ackermann_path.push_back(cur_node->state);
                    while (cur_node->parent != NULL)
                    {
                        cur_node = cur_node->parent;
                        ackermann_path.push_back(cur_node->state);
                    }
                    reverse(ackermann_path.begin(), ackermann_path.end());
                    return ackermann_path;
                }
            }

            double time_consume = (ros::Time::now()-t0).toSec();
            if ((cur_node->state.head(2)-end_state.head(2)).norm() < pos_tol)
            {
                std::cout << "[Hybrid A*] front all time: " << time_consume*1000 << " ms"<<std::endl;
                for (int i=shot_path.size()-1; i>=0; i--)
                        ackermann_path.push_back(shot_path[i]);
                ackermann_path.push_back(cur_node->state);
                while (cur_node->parent != NULL)
                {
                    cur_node = cur_node->parent;
                    ackermann_path.push_back(cur_node->state);
                }

                reverse(ackermann_path.begin(), ackermann_path.end());
                return ackermann_path;
            }
            
            if (time_consume > max_time_consume)
            {
                std::cout << "[Hybrid A*] hybrid A* time out, front all time: " << time_consume*1000 << " ms"<<std::endl;
                return ackermann_path;
            }

            open_set.pop();
            cur_node->node_state = CLOSE;
            iter_num += 1;

            Eigen::VectorXd cur_state = cur_node->state;
            Eigen::VectorXd pro_state;
            Eigen::Vector2d ctrl_input;
            std::vector<Eigen::Vector2d> inputs;

            for (double v = 0.5 * max_vel; v <= max_vel + 1e-3; v += 0.5 * max_vel)
            {
                for (double steer = -trailer->max_steer; steer <= trailer->max_steer + 1e-3; steer += 0.5 * trailer->max_steer)
                {
                    ctrl_input << v, steer;
                    inputs.push_back(ctrl_input);
                }
            }
                
            for (size_t i=0; i<inputs.size(); i++)
            {
                Eigen::Vector2d input = inputs[i];

                stateTransitVelAckermann(cur_state, input, time_interval, pro_state);
                expanded_points.points.push_back(pcl::PointXYZ(pro_state(0), pro_state(1), 0.0));

                Eigen::Vector2d pp = pro_state.head(2);
                if (!grid_map->isInMap(pp))
                    continue;

                Eigen::VectorXi pro_id;
                PathNodePtr pro_node;

                stateToIndexAckermann(pro_state, pro_id);
                pro_node = expanded_nodes.find(pro_id);

                if (pro_node != NULL && pro_node->node_state == CLOSE)
                {
                    continue;
                }

                Eigen::VectorXd xt;
                bool valid = true;
                double temp_ct = check_ratio * time_interval;
                for (double t = 0.0; t < time_interval-1e-4; t+=temp_ct)
                {
                    stateTransitVelAckermann(cur_state, input, t, xt);
                    valid = isValidAckermann(xt);
                    if (!valid)
                        break;
                }
                if (!valid)
                    continue;

                double tmp_g_score = 0.0;
                double tmp_f_score = 0.0;
                double arc = fabs(input(0)) * time_interval;
                tmp_g_score += weight_r2 * arc;
                tmp_g_score += weight_delta * fabs(input(1)) * arc;
                tmp_g_score += weight_v_change * std::fabs(input(0)-cur_node->input(0));
                tmp_g_score += weight_delta_change * std::fabs(input(1)-cur_node->input(1));
                tmp_g_score += cur_node->g_score;
                tmp_f_score = tmp_g_score + lambda_heu * tie_breaker * (pro_state.head(2) - end_state.head(2)).norm();
                

                if (pro_node == NULL)
                {
                    pro_node = path_node_pool[use_node_num];
                    pro_node->index = pro_id;
                    pro_node->state = pro_state;
                    pro_node->f_score = tmp_f_score;
                    pro_node->g_score = tmp_g_score;
                    pro_node->input = input;
                    pro_node->parent = cur_node;
                    pro_node->node_state = OPEN;
                    open_set.push(pro_node);

                    expanded_nodes.insert(pro_id, pro_node);
                    use_node_num ++;

                    if (use_node_num == allocate_num)
                    {
                        std::cout << "run out of memory." << std::endl;
                        return ackermann_path;
                    }
                }
                else if (pro_node->node_state == OPEN)
                {
                    if (tmp_g_score < pro_node->g_score)
                    {
                        pro_node->index = pro_id;
                        pro_node->f_score = tmp_f_score;
                        pro_node->g_score = tmp_g_score;
                        pro_node->input = input;
                        pro_node->parent = cur_node;
                    }
                }
            }
        }

        std::cout << "Kino Astar Failed, No path!!!" << std::endl;

        visExpanded();

        return ackermann_path;
    }

    std::vector<Eigen::VectorXd> HybridAstar::plan(const Eigen::VectorXd& start_state, const Eigen::VectorXd& end_state)
    {
        if (!set_done)
        {
            ROS_ERROR("[Hybrid A*] No Setting! Can't begin planning!");
            return front_end_path;
        }

        // reset
        int use_node_num = 0;
        int iter_num = 0;
        std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
        open_set.swap(empty_queue);
        front_end_path.clear();
        expanded_nodes.clear();
        expanded_points.clear();
        for (int i = 0; i < allocate_num; i++)
        {
            PathNodePtr node = path_node_pool[i];
            node->parent = NULL;
            node->node_state = NOT_EXPAND;
        }

        if (!isValid(start_state))
        {
            ROS_ERROR("[Hybrid A*] start is not free!!!");
            return front_end_path;
        }
        if (!isValid(end_state))
        {
            ROS_ERROR("[Hybrid A*] goal is not free!!!");
            return front_end_path;
        }

        ros::Time t0 = ros::Time::now();
        PathNodePtr cur_node = path_node_pool[0];
        cur_node->parent = NULL;
        cur_node->state = start_state;
        stateToIndex(cur_node->state, cur_node->index);
        cur_node->g_score = 0.0;
        cur_node->input = Eigen::Vector2d::Zero();
        cur_node->f_score = lambda_heu * getHeu(cur_node->state, end_state, heuristic_type);
        cur_node->node_state = OPEN;

        Eigen::VectorXi end_index;
        stateToIndex(end_state, end_index);
        Eigen::VectorXd end_se2;
        trailer->gainSE2State(end_state, end_se2);

        open_set.push(cur_node);
        use_node_num += 1;
        expanded_nodes.insert(cur_node->index, cur_node);
        
        while (!open_set.empty())
        {
            cur_node = open_set.top();

            visExpanded();
            
            if((cur_node->state.head(2) - end_state.head(2)).norm() < oneshot_range)
            {
                ros::Time t1 = ros::Time::now();
                asignShotTraj(cur_node->state, end_state);
                if (!shot_path.empty())
                {
                    std::cout << "[Hybrid A*] one-shot time: " << (ros::Time::now()-t1).toSec()*1000 << " ms"<<std::endl;
                    std::cout << "[Hybrid A*] front all time: " << (ros::Time::now()-t0).toSec()*1000 << " ms"<<std::endl;
                    retrievePath(cur_node);
                    return front_end_path;
                }
            }

            double time_consume = (ros::Time::now()-t0).toSec();
            // if (isEnd(cur_node->index, end_index))
            if (isClose(cur_node->state, end_state))
            {
                std::cout << "[Hybrid A*] front all time: " << time_consume*1000 << " ms"<<std::endl;
                retrievePath(cur_node);
                return front_end_path;
            }
            
            if (time_consume > max_time_consume)
            {
                std::cout << "[Hybrid A*] hybrid A* time out, front all time: " << time_consume*1000 << " ms"<<std::endl;
                front_end_path.clear();
                return front_end_path;
                // retrievePath(cur_node);
                // return front_end_path;
            }

            open_set.pop();
            cur_node->node_state = CLOSE;
            iter_num += 1;

            Eigen::VectorXd cur_state = cur_node->state;
            Eigen::VectorXd pro_state;
            Eigen::Vector2d ctrl_input;
            std::vector<Eigen::Vector2d> inputs;

            for (double v = 0.0; v <= max_vel + 1e-3; v += 0.5 * max_vel)
            {
                for (double steer = -trailer->max_steer; steer <= trailer->max_steer + 1e-3; steer += 0.5 * trailer->max_steer)
                {
                    ctrl_input << v, steer;
                    inputs.push_back(ctrl_input);
                }
            }
                
            for (size_t i=0; i<inputs.size(); i++)
            {
                Eigen::Vector2d input = inputs[i];

                trailer->stateTransitVel(cur_state, input, time_interval, pro_state);
                expanded_points.points.push_back(pcl::PointXYZ(pro_state(0), pro_state(1), 0.0));

                Eigen::VectorXd pro_se2_state;
                trailer->gainSE2State(pro_state, pro_se2_state);
                bool in_map = true;
                for  (size_t j=0; j<=TRAILER_NUM; j++)
                {
                    Eigen::Vector2d p = pro_se2_state.segment(j*3, 2);
                    if (!grid_map->isInMap(p))
                    {
                        in_map = false;
                        break;
                    }
                }
                if (!in_map)
                    continue;

                Eigen::VectorXi pro_id;
                PathNodePtr pro_node;

                stateToIndex(pro_state, pro_id);
                pro_node = expanded_nodes.find(pro_id);

                if (pro_node != NULL && pro_node->node_state == CLOSE)
                {
                    continue;
                }

                Eigen::VectorXd xt;
                bool valid = true;
                double temp_ct = check_ratio * time_interval;
                for (double t = 0.0; t < time_interval-1e-4; t+=temp_ct)
                {
                    trailer->stateTransitVel(cur_state, input, t, xt);
                    valid = isValid(xt);
                    // valid = isValidAckermann(xt);
                    if (!valid)
                        break;
                }
                if (!valid)
                    continue;

                double tmp_g_score = 0.0;
                double tmp_f_score = 0.0;
                double arc = fabs(input(0)) * time_interval;
                tmp_g_score += weight_r2 * arc;
                tmp_g_score += weight_delta * fabs(input(1)) * arc;
                tmp_g_score += weight_v_change * std::fabs(input(0)-cur_node->input(0));
                tmp_g_score += weight_delta_change * std::fabs(input(1)-cur_node->input(1));
                tmp_g_score += cur_node->g_score;
                tmp_f_score = tmp_g_score + lambda_heu * getHeu(pro_state, end_state, heuristic_type);

                if (pro_node == NULL)
                {
                    pro_node = path_node_pool[use_node_num];
                    pro_node->index = pro_id;
                    pro_node->state = pro_state;
                    pro_node->f_score = tmp_f_score;
                    pro_node->g_score = tmp_g_score;
                    pro_node->input = input;
                    pro_node->parent = cur_node;
                    pro_node->node_state = OPEN;
                    open_set.push(pro_node);

                    expanded_nodes.insert(pro_id, pro_node);
                    use_node_num ++;

                    if (use_node_num == allocate_num)
                    {
                        std::cout << "run out of memory." << std::endl;
                        return front_end_path;
                    }
                }
                else if (pro_node->node_state == OPEN)
                {
                    if (tmp_g_score < pro_node->g_score)
                    {
                        pro_node->index = pro_id;
                        pro_node->f_score = tmp_f_score;
                        pro_node->g_score = tmp_g_score;
                        pro_node->input = input;
                        pro_node->parent = cur_node;
                    }
                }
            }
        }

        std::cout << "Kino Astar Failed, No path!!!" << std::endl;

        visExpanded();

        return front_end_path;
    }

    void HybridAstar::visExpanded()
    {
        if (in_test)
        {
            sensor_msgs::PointCloud2 expanded_msg;
            pcl::toROSMsg(expanded_points, expanded_msg);
            expanded_msg.header.stamp = ros::Time::now();
            expanded_msg.header.frame_id = "world";
            expanded_pub.publish(expanded_msg);
            ros::Duration(0.02).sleep();
        }
    }
}