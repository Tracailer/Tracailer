#pragma once

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <queue>
#include <algorithm>
#include <boost/functional/hash.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>

#include "planner/grid_map.h"
#include "planner/trailer.hpp"

#define CLOSE 'a'
#define OPEN 'b'
#define NOT_EXPAND 'c'

using namespace std;

namespace trailer_planner
{
    class PathNode
    {
        public:
            PathNode* parent;
            char node_state;
            Eigen::VectorXi index;
            Eigen::VectorXd state;
            Eigen::Vector2d input;
            double g_score;
            double f_score;
            PathNode(): parent(nullptr), node_state(NOT_EXPAND) {}
            ~PathNode() {}
    };
    typedef PathNode* PathNodePtr;

    class NodeComparator
    {
        public:
            template <class NodePtr>
            bool operator()(NodePtr node1, NodePtr node2) 
            {
                return node1->f_score > node2->f_score;
            }
    };

    template <typename T>
    struct matrix_hash : std::unary_function<T, size_t> 
    {
        std::size_t operator()(T const& matrix) const 
        {
            size_t seed = 0;
            for (long int i = 0; i < matrix.size(); ++i)
            {
                auto elem = *(matrix.data() + i);
                seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };

    template <class NodePtr>
    class NodeHashTable 
    {
        private:
            std::unordered_map<Eigen::VectorXi, NodePtr, matrix_hash<Eigen::VectorXi>> data_;

        public:
            NodeHashTable() {}
            ~NodeHashTable() {}

            void insert(Eigen::VectorXi idx, NodePtr node)
            {
                data_.insert(std::make_pair(idx, node));
            }

            NodePtr find(Eigen::VectorXi idx) 
            {
                auto iter = data_.find(idx);
                return iter == data_.end() ? NULL : iter->second;
            }

            void clear() { data_.clear(); }
    };


    class HybridAstar
    {
        private:
            // datas
            bool set_done = false;
            Trailer::Ptr trailer;
            GridMap::Ptr grid_map;
            std::vector<PathNodePtr> path_node_pool;
            NodeHashTable<PathNodePtr> expanded_nodes;
            pcl::PointCloud<pcl::PointXYZ> expanded_points;
            std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> open_set;
            ompl::base::StateSpacePtr shot_finder;
            std::vector<Eigen::VectorXd> front_end_path;
            std::vector<Eigen::VectorXd> shot_path;
            
            // params
            int    allocate_num;
            int    heuristic_type;
            bool   in_test;
            double yaw_resolution, yaw_resolution_inv;
            double lambda_heu;
            double weight_r2;
            double weight_delta;
            double weight_v_change;
            double weight_delta_change;
            double time_interval;
            double oneshot_range;
            double check_ratio;
            double max_vel;
            double pos_tol;
            double theta_tol;
            double max_time_consume;
            double tie_breaker = 1.0 + 1.0 / 10000;

            // ros
            ros::Publisher expanded_pub;
            Eigen::VectorXd start_pos;
            Eigen::VectorXd target_pos;

        public:
            HybridAstar() {}
            ~HybridAstar()
            {
                for (int i = 0; i < allocate_num; i++)
                    delete path_node_pool[i];
            }
            
            void init(ros::NodeHandle& nh);
            void visExpanded();
            std::vector<Eigen::VectorXd> plan(const Eigen::VectorXd& start_state, const Eigen::VectorXd& end_state);
            std::vector<Eigen::VectorXd> planAckermann(const Eigen::Vector3d& start_state, const Eigen::Vector3d& end_state);
            std::vector<Eigen::VectorXd> pureAstarPlan(const Eigen::VectorXd& start_state, const Eigen::VectorXd& end_state);

            inline void setTrailerEnv(Trailer::Ptr t, GridMap::Ptr g);
            inline int yawToIndex(const double &yaw);
            inline void stateToIndex(const Eigen::VectorXd& state, Eigen::VectorXi& idx); 
            inline void stateToIndexAckermann(const Eigen::VectorXd& state, Eigen::VectorXi& idx);
            inline bool isValid(const Eigen::VectorXd& state);
            inline bool isValidAckermann(const Eigen::VectorXd& state);
            inline bool isClose(const Eigen::VectorXd& state, const Eigen::VectorXd& end);
            inline bool isEnd(const Eigen::VectorXi& state, const Eigen::VectorXi& end);
            inline double normalizedAngle(const double &angle);
            // type=0: simple; type=1: Libai
            inline double getHeu(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, int type);
            inline void asignShotTraj(const Eigen::VectorXd &state1, const Eigen::VectorXd &state2);
            inline void asignShotTrajAckermann(const Eigen::VectorXd &state1, const Eigen::VectorXd &state2);
            inline void retrievePath(PathNodePtr end_node);
            inline void stateTransitVelAckermann(const Eigen::VectorXd& state0,
                                                const Eigen::Vector2d& input,
                                                const double& T,
                                                Eigen::VectorXd& state1);

            typedef shared_ptr<HybridAstar> Ptr;
            typedef unique_ptr<HybridAstar> UniPtr;
    };
        
    inline void HybridAstar::setTrailerEnv(Trailer::Ptr t, GridMap::Ptr g)
    {
        if (set_done)
            return;

        trailer = t;
        shot_finder = std::make_shared<ompl::base::DubinsStateSpace>(trailer->wheel_base / tan(trailer->max_steer));

        grid_map = g;
        allocate_num = g->getGridNum() * yaw_resolution_inv * PI_X_2 * TRAILER_NUM * 2;
        path_node_pool.resize(allocate_num);
        for (int i = 0; i < allocate_num; i++)
        {
            path_node_pool[i] = new PathNode;
        }

        set_done = true;
        
        return;
    }

    inline int HybridAstar::yawToIndex(const double &yaw)
    {
        double nor_yaw = normalizedAngle(yaw);
        int idx = floor((nor_yaw + M_PI) * yaw_resolution_inv);
        return idx;
    }
    
    inline void HybridAstar::stateToIndex(const Eigen::VectorXd& state, Eigen::VectorXi& idx)
    {
        Eigen::Vector2i sidx;
        grid_map->posToIndex(state.head(2), sidx);
        idx.resize(state.size());
        idx.head(2) = sidx;
        for (size_t i=0; i<=TRAILER_NUM; i++)
            idx(i+2) = yawToIndex(state(i+2));
        return;
    }

    inline void HybridAstar::stateToIndexAckermann(const Eigen::VectorXd& state, Eigen::VectorXi& idx)
    {
        Eigen::Vector2i sidx;
        grid_map->posToIndex(state.head(2), sidx);
        idx.resize(3);
        idx.head(2) = sidx;
        idx(2) = yawToIndex(state(2));
        return;
    }

    inline bool HybridAstar::isValid(const Eigen::VectorXd& state)
    {
        // Jack Knife
        Eigen::VectorXd se2_state;
        if ((size_t)state.size() == TRAILER_NUM + 3)
        {
            if (trailer->isJackKnife(state))
                return false;
            trailer->gainSE2State(state, se2_state);
        }
        else if ((size_t)state.size() == 3*TRAILER_NUM + 3)
        {
            if (trailer->isJackKnifeSE2(state))
                return false;
            se2_state = state;
        }          
        
        // Collision
        bool is_occ = false;
        for (size_t i=0; i<=TRAILER_NUM; i++)
        {
            Eigen::Vector2d p = se2_state.segment(3*i, 2);
            Eigen::Matrix2d rotM;
            double yaw = se2_state(3*i+2);
            rotM << cos(yaw), -sin(yaw),
                  sin(yaw),  cos(yaw);
            vector<Eigen::Vector2d> points;
            double x_l = trailer->length[i]*0.5 + 0.05;
            double y_l = trailer->width*0.5 + 0.05;
            if (i==0)
                x_l = trailer->length[i] - trailer->rear_length + 0.05;
            points.push_back(p+rotM*Eigen::Vector2d(x_l, y_l));
            points.push_back(p+rotM*Eigen::Vector2d(x_l, -y_l));
            if (i==0)
                x_l = trailer->rear_length + 0.05;
            points.push_back(p+rotM*Eigen::Vector2d(-x_l, -y_l));
            points.push_back(p+rotM*Eigen::Vector2d(-x_l, y_l));
            for(int j = 0; j < 4; j++)
            {
                if (grid_map->isLineOccupancy(points[j], points[(j+1)%points.size()]))
                {
                    is_occ = true;
                    break;
                }
            }
            if (is_occ)
                break;
        }

        return !is_occ;
    }

    inline bool HybridAstar::isValidAckermann(const Eigen::VectorXd& state)
    {
        bool is_occ = false;
        Eigen::Matrix2d rotM;
        Eigen::Vector2d p = state.head(2);
        double yaw = state(2);
        rotM << cos(yaw), -sin(yaw),
                sin(yaw),  cos(yaw);
        vector<Eigen::Vector2d> points;
        double y_l = trailer->width*0.5 + 0.05;
        double x_l = trailer->length[0] - trailer->rear_length + 0.05;
        points.push_back(p+rotM*Eigen::Vector2d(x_l, y_l));
        points.push_back(p+rotM*Eigen::Vector2d(x_l, -y_l));
        x_l = trailer->rear_length + 0.05;
        points.push_back(p+rotM*Eigen::Vector2d(-x_l, -y_l));
        points.push_back(p+rotM*Eigen::Vector2d(-x_l, y_l));
        for(int j = 0; j < 4; j++)
        {
            if (grid_map->isLineOccupancy(points[j], points[(j+1)%points.size()]))
            {
                is_occ = true;
                break;
            }
        }
        return !is_occ;
    }

    inline bool HybridAstar::isClose(const Eigen::VectorXd& state, const Eigen::VectorXd& end)
    {
        bool close = true;
        if ((state.head(2)-end.head(2)).norm() > pos_tol)
            close = false;
        else
            for (size_t i=0; i<=TRAILER_NUM; i++)
            {
                if (fabs(normalizedAngle(state(i+2)-end(i+2))) > theta_tol)
                {
                    close = false;
                    break;
                }
            }
        return close;
    }

    inline bool HybridAstar::isEnd(const Eigen::VectorXi& state, const Eigen::VectorXi& end)
    {
        bool is_end = true;
        for (int i=0; i<state.size(); i++)
            if (state[i]!=end[i])
            {
                is_end = false;
                break;
            }
        return is_end;
    }

    inline double HybridAstar::normalizedAngle(const double &angle)
    {
        double nor_angle = angle;

        while (nor_angle>M_PI)
            nor_angle -= PI_X_2;

        while (nor_angle<-M_PI)
            nor_angle += PI_X_2;

        return nor_angle;
    }

    inline void static normalizeAngle(const double &ref_angle, double &angle)
    {
        while(ref_angle - angle > M_PI)
            angle += 2*M_PI;
        while(ref_angle - angle < -M_PI)
            angle -= 2*M_PI;
        return;
    }

    // type == 0: simple Euclid distance; type == 1: Libai
    inline double HybridAstar::getHeu(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, int type)
    {
        assert(x1.size() == (long int)TRAILER_NUM+3);

        if (type==0)
            return tie_breaker * (x1.head(2) - x2.head(2)).norm();
        else if (type==1)
        {
            // get RS path
            std::pair<std::vector<Eigen::Vector2d>, double> astar_res = grid_map->astarPlan(x1.head(2), x2.head(2));
            ompl::base::ScopedState<> from(shot_finder), to(shot_finder), s(shot_finder);
            from[0] = x1[0]; from[1] = x1[1]; from[2] = x1[2];
            to[0] = x2[0]; to[1] = x2[1]; to[2] = x2[2];
            double rs_len = shot_finder->distance(from(), to());

            std::vector<double> reals;
            std::vector<Eigen::Vector3d> path;
            for (double l = 0.0; l <=rs_len; l += 0.5*time_interval)
            {
                shot_finder->interpolate(from(), to(), l/rs_len, s());
                reals = s.reals();
                path.push_back(Eigen::Vector3d(reals[0], reals[1], reals[2]));        
            }

            // estimate end states
            Eigen::VectorXd end_state = x1;
            Eigen::VectorXd dtheta;
            dtheta.resize(TRAILER_NUM+1);
            dtheta.setZero();
            for (size_t i=1; i<path.size(); i++)
            {
                double vx = fabs((path[i].x()-path[i-1].x()) / cos(path[i].z()));
                double vy = fabs((path[i].y()-path[i-1].y()) / sin(path[i].z()));
                double v = vx > vy ? vy : vx;
                
                dtheta(0) = normalizedAngle(path[i].z() - path[i-1].z());
                for (size_t j=0; j<TRAILER_NUM; j++)
                {
                    double sthetad = sin(end_state(j+2)-end_state(j+3));
                    double cthetad = cos(end_state(j+2)-end_state(j+3));
                    dtheta(j+1) = v * sthetad / trailer->Lhead[j];
                    end_state(j+3) = normalizedAngle(end_state(j+3) + dtheta(j+1));
                    v = v * cthetad;
                }
                end_state(2) = path[i].z();
            }
            end_state.head(2) = x2.head(2);
            
            Eigen::VectorXd x2_se2;
            Eigen::VectorXd se2_end;
            trailer->gainSE2State(x2, x2_se2);
            trailer->gainSE2State(end_state, se2_end);
            
            // compute H
            double sum_euclid = 0.0;
            for (size_t i=0; i<TRAILER_NUM; i++)
            {
                sum_euclid += (x2_se2.segment(3*i+3, 2) - se2_end.segment(3*i+3, 2)).norm();
            }
            return max(astar_res.second, sum_euclid+rs_len);
        }

        return 0.0;
    }

    inline void HybridAstar::asignShotTraj(const Eigen::VectorXd &state1, const Eigen::VectorXd &state2)
    {
        std::vector<Eigen::Vector3d> path;
        ompl::base::ScopedState<> from(shot_finder), to(shot_finder), s(shot_finder);
        from[0] = state1[0]; from[1] = state1[1]; from[2] = state1[2];
        to[0] = state2[0]; to[1] = state2[1]; to[2] = state2[2];
        std::vector<double> reals;
        double len = shot_finder->distance(from(), to());

        for (double l = 0.0; l <=len; l += check_ratio*time_interval)
        {
            shot_finder->interpolate(from(), to(), l/len, s());
            reals = s.reals();
            path.push_back(Eigen::Vector3d(reals[0], reals[1], reals[2]));        
        }

        Eigen::VectorXd end_state = state1;
        Eigen::VectorXd dtheta;
        dtheta.resize(TRAILER_NUM+1);
        dtheta.setZero();
        shot_path.clear();
        for (size_t i=1; i<path.size(); i++)
        {
            double vx = fabs((path[i].x()-path[i-1].x()) / cos(path[i].z()));
            double vy = fabs((path[i].y()-path[i-1].y()) / sin(path[i].z()));
            double v = vx > vy ? vy : vx;

            dtheta(0) = normalizedAngle(path[i].z() - path[i-1].z());
            for (size_t j=0; j<TRAILER_NUM; j++)
            {
                double sthetad = sin(end_state(j+2)-end_state(j+3));
                double cthetad = cos(end_state(j+2)-end_state(j+3));
                dtheta(j+1) = v * sthetad / trailer->Lhead[j];
                end_state(j+3) = normalizedAngle(end_state(j+3) + dtheta(j+1));
                v = v * cthetad;
            }
            end_state.head(3) = path[i].head(3);
            if (isValid(end_state))
            {
                shot_path.push_back(end_state);
            }
            else
            {
                shot_path.clear();
                break;
            }
        }
        if (!shot_path.empty() && !isClose(shot_path[shot_path.size()-1], state2))
            shot_path.clear();

        return;
    }

    inline void HybridAstar::asignShotTrajAckermann(const Eigen::VectorXd &state1, const Eigen::VectorXd &state2)
    {
        std::vector<Eigen::Vector3d> path;
        ompl::base::ScopedState<> from(shot_finder), to(shot_finder), s(shot_finder);
        from[0] = state1[0]; from[1] = state1[1]; from[2] = state1[2];
        to[0] = state2[0]; to[1] = state2[1]; to[2] = state2[2];
        std::vector<double> reals;
        double len = shot_finder->distance(from(), to());

        for (double l = 0.0; l <=len; l += check_ratio*time_interval)
        {
            shot_finder->interpolate(from(), to(), l/len, s());
            reals = s.reals();
            path.push_back(Eigen::Vector3d(reals[0], reals[1], reals[2]));        
        }

        shot_path.clear();
        Eigen::VectorXd temp_state;
        temp_state.resize(3);
        for (size_t i=1; i<path.size(); i++)
        {
            temp_state = path[i];
            if (isValidAckermann(temp_state))
            {
                shot_path.push_back(temp_state);
            }
            else
            {
                shot_path.clear();
                break;
            }
        }

        return;
    }

    inline void HybridAstar::retrievePath(PathNodePtr end_node)
    {
        for (int i=shot_path.size()-1; i>=0; i--)
        {
            front_end_path.push_back(shot_path[i]);
        }

        PathNodePtr cur_node = end_node;
        front_end_path.push_back(cur_node->state);

        while (cur_node->parent != NULL)
        {
            cur_node = cur_node->parent;
            front_end_path.push_back(cur_node->state);
        }

        reverse(front_end_path.begin(), front_end_path.end());

        return;
    }

    inline void HybridAstar::stateTransitVelAckermann(const Eigen::VectorXd& state0,
                                        const Eigen::Vector2d& input,
                                        const double& T,
                                        Eigen::VectorXd& state1)
    {
        double v = input(0);
        double w = v * tan(input(1)) / trailer->wheel_base;
        double y = T * w;

        state1.resize(state0.size());
        if (fabs(w) > 1e-4)
        {
            state1(0) = state0(0) + v / w * (sin(state0(2)+y) - sin(state0(2)));
            state1(1) = state0(1) - v / w * (cos(state0(2)+y) - cos(state0(2)));
            state1(2) = state0(2) + y;
            trailer->normYaw(state1(2));
        }
        else
        {
            state1(0) = state0(0) + v * T * cos(state0(2));
            state1(1) = state0(1) + v * T * sin(state0(2));
            state1(2) = state0(2);
        }
        return;
    }
}
