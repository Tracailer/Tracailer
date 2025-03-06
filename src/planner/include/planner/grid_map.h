#pragma once

#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <random>

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define IN_CLOSE 97
#define IN_OPEN 98
#define IS_UNKNOWN 99

using namespace std;

namespace trailer_planner
{
    struct GridNode;
    typedef GridNode* GridNodePtr;

    struct GridNode
    {   
        Eigen::Vector2i index;
        double fScore;
        double gScore;
        GridNodePtr parent;
        char id;

        GridNode() : 
            index(Eigen::Vector2i::Zero()), 
            fScore(0.0), 
            gScore(0.0), 
            parent(nullptr), 
            id(IS_UNKNOWN) {}
        
        GridNode(const Eigen::Vector2i& _index) : 
            index(_index), 
            fScore(0.0), 
            gScore(0.0), 
            parent(nullptr), 
            id(IS_UNKNOWN) {}

        void reset()
        {
            parent = nullptr;
            gScore = fScore = 0.0;
            id = IS_UNKNOWN;
            return;
        }
        
        ~GridNode(){};
    };

    class GridMap
    {
        public:
            // params
            int             buffer_size;
            double          resolution, resolution_inv;
            Eigen::Vector2d map_origin;
            Eigen::Vector2d map_size;
            Eigen::Vector2d min_boundary;
            Eigen::Vector2d max_boundary;
            Eigen::Vector2i min_idx;
            Eigen::Vector2i max_idx;
            Eigen::Vector2i voxel_num;
            double          custom_resolution, custom_resolution_inv;
            Eigen::Vector2i custom_voxel_num;
            vector<double>  custom_buffer;
            Eigen::MatrixXd local_pts;

        private:
            //datas
            GridNodePtr* grid_node_map = nullptr;
            vector<char>   occ_buffer;
            vector<double> esdf_buffer;
            sensor_msgs::PointCloud2 esdf_cloud;

            //ros
            bool       map_ready = false;
            ros::Timer vis_timer;
            ros::Publisher esdf_pub;
            ros::Subscriber cloud_sub;

        public:
            GridMap() {}
            ~GridMap()
            { 
                for (int i=0; i<buffer_size; i++) 
                    delete[] grid_node_map[i];

                delete[] grid_node_map;
                return;
            }

            void init(ros::NodeHandle& nh);
            void updateESDF2d();
            void visCallback(const ros::TimerEvent& /*event*/);
            void cloudCallback(const sensor_msgs::PointCloud2 msg);
            std::pair<std::vector<Eigen::Vector2d>, double> astarPlan(const Eigen::Vector2d& start, const Eigen::Vector2d& end);

            inline void getDistance(const Eigen::Vector2d& pos, double& distance);
            inline void getDisWithGradI(const Eigen::Vector2d& pos, double& distance, Eigen::Vector2d& grad);
            inline bool isCollision(const Eigen::Vector2d& pos);
            inline void boundIndex(Eigen::Vector2i& id);
            inline void posToIndex(const Eigen::Vector2d& pos, Eigen::Vector2i& id);
            inline void indexToPos(const Eigen::Vector2i& id, Eigen::Vector2d& pos);
            inline int toAddress(const Eigen::Vector2i& id);
            inline int toAddress(const int& x, const int& y);
            inline bool isInMap(const Eigen::Vector2d& pos);
            inline bool isInMap(const Eigen::Vector2i& idx);
            inline bool isOccupancy(const Eigen::Vector2d& pos);
            inline bool isOccupancy(const Eigen::Vector2i& id);
            inline bool isLineOccupancy(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);
            inline bool isLineOccupancyWithRadius(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, double radius);
            inline int getGridNum();
            inline double getResolution();
            inline bool mapReady();

            inline void setCustomBuffer(const double& res);
            inline void getCustomPoint(Eigen::Vector2d& pos);
            inline void getCustomValue(const Eigen::Vector2d& pos, double& value);
            inline void setCustomValue(const Eigen::Vector2d& pos, const double& value);

            typedef shared_ptr<GridMap> Ptr;
            typedef unique_ptr<GridMap> UniPtr;
    };

    inline void GridMap::getDistance(const Eigen::Vector2d& pos, double& distance)
    {
        if (!isInMap(pos))
        {
            distance = 1e+10;
            return;
        }

        /* use trilinear interpolation */
        Eigen::Vector2d pos_m = pos;
        pos_m(0) -= 0.5 * resolution;
        pos_m(1) -= 0.5 * resolution;

        Eigen::Vector2i idx;
        posToIndex(pos_m, idx);

        Eigen::Vector2d idx_pos;
        indexToPos(idx, idx_pos);

        Eigen::Vector2d diff = pos - idx_pos;
        diff(0) *= resolution_inv;
        diff(1) *= resolution_inv;

        double values[2][2];
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
            {
                Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
                boundIndex(current_idx);
                values[x][y] = esdf_buffer[toAddress(current_idx)];
            }
        
        // value & grad
        double v0 = values[0][0] * (1 - diff[0]) + values[1][0] * diff[0];
        double v1 = values[0][1] * (1 - diff[0]) + values[1][1] * diff[0];
        distance = v0 * (1 - diff[1]) + v1 * diff[1];

        return;
    }

    inline void GridMap::getDisWithGradI(const Eigen::Vector2d& pos, double& distance, Eigen::Vector2d& grad)
    {
        if (!isInMap(pos))
        {
            distance = 0.0;
            grad.setZero();
            return;
        }

        /* use trilinear interpolation */
        Eigen::Vector2d pos_m = pos;
        pos_m(0) -= 0.5 * resolution;
        pos_m(1) -= 0.5 * resolution;

        Eigen::Vector2i idx;
        posToIndex(pos_m, idx);

        Eigen::Vector2d idx_pos;
        indexToPos(idx, idx_pos);

        Eigen::Vector2d diff = pos - idx_pos;
        diff(0) *= resolution_inv;
        diff(1) *= resolution_inv;

        double values[2][2];
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
            {
                Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
                boundIndex(current_idx);
                values[x][y] = esdf_buffer[toAddress(current_idx)];
            }
        
        // value & grad
        double v0 = values[0][0] * (1 - diff[0]) + values[1][0] * diff[0];
        double v1 = values[0][1] * (1 - diff[0]) + values[1][1] * diff[0];
        distance = v0 * (1 - diff[1]) + v1 * diff[1];
        grad(1) = (v1 - v0) * resolution_inv;
        grad(0) = (1 - diff[1]) * (values[1][1] - values[0][1]);
        grad(0) += diff[1] * (values[1][0] - values[0][0]);
        grad(0) *= resolution_inv;

        return;
    }

    inline bool GridMap::isCollision(const Eigen::Vector2d& pos)
    {
        if (isInMap(pos))
        {
            double dist;
            getDistance(pos, dist);
            return dist < 0.0;
        }
        else
            return true;
    }

    inline void GridMap::boundIndex(Eigen::Vector2i& id)
    {
        id(0) = max(min(id(0), max_idx(0)), min_idx(0));
        id(1) = max(min(id(1), max_idx(1)), min_idx(1));

        return;
    }

    inline void GridMap::posToIndex(const Eigen::Vector2d& pos, Eigen::Vector2i& id)
    {
        id(0) = floor((pos(0) - map_origin(0)) * resolution_inv);
        id(1) = floor((pos(1) - map_origin(1)) * resolution_inv);
        return;
    }

    inline void GridMap::indexToPos(const Eigen::Vector2i& id, Eigen::Vector2d& pos)
    {
        pos(0) = (id(0) + 0.5) * resolution  + map_origin(0);
        pos(1) = (id(1) + 0.5) * resolution  + map_origin(1);
        return;
    }

    inline int GridMap::toAddress(const Eigen::Vector2i& id) 
    {
        return id(0) * voxel_num(1) + id(1);
    }

    inline int GridMap::toAddress(const int& x, const int& y) 
    {
        return x * voxel_num(1) + y;
    }
    
    inline bool GridMap::isInMap(const Eigen::Vector2d& pos) 
    {
        if (pos(0) < min_boundary(0) + 1e-4 || \
            pos(1) < min_boundary(1) + 1e-4     ) 
        {
            return false;
        }

        if (pos(0) > max_boundary(0) - 1e-4 || \
            pos(1) > max_boundary(1) - 1e-4     ) 
        {
            return false;
        }

        return true;
    }

    inline bool GridMap::isInMap(const Eigen::Vector2i& idx)
    {
        if (idx(0) < 0 || idx(1) < 0)
        {
            return false;
        }

        if (idx(0) > voxel_num(0) - 1 || \
            idx(1) > voxel_num(1) - 1     ) 
        {
            return false;
        }

        return true;
    }

    inline bool GridMap::isOccupancy(const Eigen::Vector2d& pos)
    {
        Eigen::Vector2i id;

        posToIndex(pos, id);
        
        return isOccupancy(id);
    }

    inline bool GridMap::isOccupancy(const Eigen::Vector2i& id)
    {
        if (!isInMap(id))
            return true;

        return occ_buffer[toAddress(id)] == 1;
    }

    inline bool GridMap::isLineOccupancy(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2)
    {
        bool occ = false;
        Eigen::Vector2d diff = p2 - p1;
        int max_diff = (diff * resolution_inv).lpNorm<Eigen::Infinity>() / 0.8;
        Eigen::Vector2d step = diff / (1.0 * max_diff);

        for (int i=1; i<max_diff; i++)
        {
            Eigen::Vector2d pt = p1 + step * i;
            if (isOccupancy(pt))
            {
                occ = true;
                break;
            }
        }

        return occ;
    }

    inline bool GridMap::isLineOccupancyWithRadius(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, double radius)
    {
        bool occ = false;
        Eigen::Vector2d diff = p2 - p1;
        int max_diff = (diff * resolution_inv).lpNorm<Eigen::Infinity>() / 0.8;
        Eigen::Vector2d step = diff / (1.0 * max_diff);

        for (int i=1; i<max_diff; i++)
        {
            Eigen::Vector2d pt = p1 + step * i;
            double d;
            getDistance(pt, d);
            if (d < radius)
            // if (isOccupancy(pt))
            {
                occ = true;
                break;
            }
        }

        return occ;
    }

    inline int GridMap::getGridNum()
    {
        return voxel_num(0)*voxel_num(1);
    }

    inline double GridMap::getResolution()
    {
        return resolution;
    }

    inline bool GridMap::mapReady()
    {
        return map_ready;
    }

    inline void GridMap::setCustomBuffer(const double& res)
    {
        custom_resolution = res;
        custom_resolution_inv = 1.0 / res;
        custom_buffer.clear();
        custom_voxel_num(0) = ceil(map_size(0) / custom_resolution);
        custom_voxel_num(1) = ceil(map_size(1) / custom_resolution);
        custom_buffer = vector<double>(custom_voxel_num(0) * custom_voxel_num(1), 0.0);
        return;
    }

    inline void GridMap::getCustomPoint(Eigen::Vector2d& pos)
    {
        Eigen::Vector2i idx;
        idx(0) = floor((pos(0) - map_origin(0)) * custom_resolution_inv);
        idx(1) = floor((pos(1) - map_origin(1)) * custom_resolution_inv);
        pos(0) = (idx(0) + 0.5) * custom_resolution  + map_origin(0);
        pos(1) = (idx(1) + 0.5) * custom_resolution  + map_origin(1);
        return;
    }

    inline void GridMap::getCustomValue(const Eigen::Vector2d& pos, double& value)
    {
        if (!isInMap(pos))
        {
            std::cout << "out of map" << std::endl;
            return;
        }
        Eigen::Vector2i idx;
        idx(0) = floor((pos(0) - map_origin(0)) * custom_resolution_inv);
        idx(1) = floor((pos(1) - map_origin(1)) * custom_resolution_inv);
        value = custom_buffer[idx(0) * custom_voxel_num(1) + idx(1)];
        return;
    }

    inline void GridMap::setCustomValue(const Eigen::Vector2d& pos, const double& value)
    {
        if (!isInMap(pos))
        {
            std::cout << "out of map" << std::endl;
            return;
        }
        Eigen::Vector2i idx;
        idx(0) = floor((pos(0) - map_origin(0)) * custom_resolution_inv);
        idx(1) = floor((pos(1) - map_origin(1)) * custom_resolution_inv);
        custom_buffer[idx(0) * custom_voxel_num(1) + idx(1)] = value;
        return;
    }
}