#include "planner/grid_map.h"

namespace trailer_planner
{
    void GridMap::init(ros::NodeHandle& nh)
    {
        nh.getParam("grid_map/map_size_x", map_size[0]);
        nh.getParam("grid_map/map_size_y", map_size[1]);
        nh.getParam("grid_map/resolution", resolution);

        esdf_pub = nh.advertise<sensor_msgs::PointCloud2>("/esdf_map", 1);
        // cloud_sub = nh.subscribe("/local_map", 1, &GridMap::cloudCallback, this);
        cloud_sub = nh.subscribe("/global_map", 1, &GridMap::cloudCallback, this);
        vis_timer = nh.createTimer(ros::Duration(1.0), &GridMap::visCallback, this);
        
        // origin and boundary
        min_boundary = -map_size / 2.0;
        max_boundary = map_size / 2.0;
        map_origin = min_boundary;

        // resolution
        resolution_inv = 1.0 / resolution;

        // voxel num
        voxel_num(0) = ceil(map_size(0) / resolution);
        voxel_num(1) = ceil(map_size(1) / resolution);

        // idx
        min_idx = Eigen::Vector2i::Zero();
        max_idx = voxel_num - Eigen::Vector2i::Ones();

        // datas
        buffer_size  = voxel_num(0) * voxel_num(1);
        esdf_buffer = vector<double>(buffer_size, 0.0);
        occ_buffer = vector<char>(buffer_size, 0);
        custom_buffer = vector<double>(buffer_size, 0.0);
        grid_node_map = new GridNodePtr[buffer_size];
        for (int i=0; i<buffer_size; i++)
            grid_node_map[i] = new GridNode();
        map_ready = false;

        return;
    }

    template <typename F_get_val, typename F_set_val>
    void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int size) 
    {
        int v[size];
        double z[size + 1];

        int k = start;
        v[start] = start;
        z[start] = -std::numeric_limits<double>::max();
        z[start + 1] = std::numeric_limits<double>::max();

        for (int q = start + 1; q <= end; q++) {
            k++;
            double s;

            do {
                k--;
                s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
            } while (s <= z[k]);

            k++;

            v[k] = q;
            z[k] = s;
            z[k + 1] = std::numeric_limits<double>::max();
        }

        k = start;

        for (int q = start; q <= end; q++) {
            while (z[k + 1] < q) k++;
            double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
            f_set_val(q, val);
        }
    }

    void GridMap::updateESDF2d()
    {
        int rows = voxel_num[0];
        int cols = voxel_num[1];

        Eigen::MatrixXd tmp_buffer;
        Eigen::MatrixXd neg_buffer;
        Eigen::MatrixXi neg_map;
        Eigen::MatrixXd dist_buffer;
        tmp_buffer.resize(rows, cols);
        neg_buffer.resize(rows, cols);
        neg_map.resize(rows, cols);
        dist_buffer.resize(rows, cols);

        /* ========== compute positive DT ========== */

        for (int x = min_idx[0]; x <= max_idx[0]; x++)
        {
            fillESDF(
                [&](int y)
                {
                    return occ_buffer[toAddress(x, y)] == 1 ?
                        0 :
                        std::numeric_limits<double>::max();
                },
                [&](int y, double val) { tmp_buffer(x, y) = val; }, min_idx[1],
                max_idx[1], cols
            );
        }

        for (int y = min_idx[1]; y <= max_idx[1]; y++) {
            fillESDF(
                [&](int x) { return tmp_buffer(x, y); },
                [&](int x, double val)
                {
                    dist_buffer(x, y) = resolution * std::sqrt(val);
                },
                min_idx[0], max_idx[0], rows
            );
        }

        /* ========== compute negative distance ========== */
        for (int x = min_idx(0); x <= max_idx(0); ++x)
            for (int y = min_idx(1); y <= max_idx(1); ++y)
            {
                if (occ_buffer[toAddress(x, y)] == 0)
                {
                    neg_map(x, y) = 1;
                } else if (occ_buffer[toAddress(x, y)] == 1)
                {
                    neg_map(x, y) = 0;
                } else 
                {
                    ROS_ERROR("what?");
                }
            }

        for (int x = min_idx[0]; x <= max_idx[0]; x++) {
            fillESDF(
                [&](int y)
                {
                    return neg_map(x, y) == 1 ?
                        0 :
                        std::numeric_limits<double>::max();
                },
                [&](int y, double val) { tmp_buffer(x, y) = val; }, min_idx[1],
                max_idx[1], cols
            );
        }

        for (int y = min_idx[1]; y <= max_idx[1]; y++)
        {
            fillESDF(
                [&](int x) { return tmp_buffer(x, y); },
                [&](int x, double val)
                {
                    neg_buffer(x, y) = resolution * std::sqrt(val);
                },
                min_idx[0], max_idx[0], rows
            );
        }

        /* ========== combine pos and neg DT ========== */
        for (int x = min_idx(0); x <= max_idx(0); ++x)
            for (int y = min_idx(1); y <= max_idx(1); ++y)
            {
                esdf_buffer[toAddress(x, y)] = dist_buffer(x, y);
                if (neg_buffer(x, y) > 0.0)
                    esdf_buffer[toAddress(x, y)] += (-neg_buffer(x, y) + resolution);
            }
        
        return;
    }

    void GridMap::cloudCallback(const sensor_msgs::PointCloud2 msg)
    {
        // if (map_ready)
        //     return;
	    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
        
        pcl::PointCloud<pcl::PointXYZI> pc;
        pcl::fromROSMsg(msg, pc);

        std::vector<Eigen::Vector2d> local_p;
        int idx = 0;
        for (size_t i=0; i<pc.points.size(); i++)
        {
            Eigen::Vector2i id;
            posToIndex(Eigen::Vector2d(pc.points[i].x, pc.points[i].y), id);
            if (isInMap(id))
            {
                occ_buffer[toAddress(id)] = 1;
                if (idx < 100)
                {
                    local_p.push_back(Eigen::Vector2d(pc.points[i].x, pc.points[i].y));
                    idx++;
                }  
            }
        }
        int pts_size = min((int)local_p.size(), 100);
        local_pts.resize(2, pts_size);
        for (int i=0; i<pts_size; i++)
            local_pts.col(i) = local_p[i];

        updateESDF2d();
        pc.clear();
        // test esdf computation
        for (int i=0; i<voxel_num(0); i++)
        {
            for (int j=0; j<voxel_num(1); j++)
            {
                Eigen::Vector2i id(i, j);
                Eigen::Vector2d pos;
                indexToPos(id, pos);
                pcl::PointXYZI p;
                p.x = pos(0);
                p.y = pos(1);
                p.z = esdf_buffer[toAddress(id)];
                p.intensity = 1.0;
                pc.push_back(p);
            }
        }
        // test interpolation 
        // for (double i=min_boundary(0); i<max_boundary(0); i+=0.2*resolution)
        // {
        //     for (double j=min_boundary(1); j<max_boundary(1); j+=0.2*resolution)
        //     {
        //         Eigen::Vector2d pos(i, j);
        //         Eigen::Vector2d grad;
        //         double dist = 0.0;
        //         getDisWithGradI(pos, dist, grad);
        //         pcl::PointXYZI p;
        //         p.x = pos(0);
        //         p.y = pos(1);
        //         p.z = 0.0;
        //         p.intensity = dist;
        //         pc.push_back(p);
        //     }
        // }
        pc.header.frame_id = "world";
        pc.width = pc.points.size();
        pc.height = 1;
        pc.is_dense = true;
        pcl::toROSMsg(pc, esdf_cloud);

        map_ready = true;
        
        return;
    }

    void GridMap::visCallback(const ros::TimerEvent& /*event*/)
    {
        if (!map_ready)
            return;
        esdf_pub.publish(esdf_cloud);
        return;
    }

    std::pair<std::vector<Eigen::Vector2d>, double> GridMap::astarPlan(const Eigen::Vector2d& start, 
                                                                        const Eigen::Vector2d& end)
    {
        std::vector<Eigen::Vector2d> path;

        if (!map_ready)
            return make_pair(path, 0.0);

        for (int i=0; i<buffer_size; i++)
        {
            grid_node_map[i]->reset();
        }

        if(!isInMap(start) || !isInMap(end))
        {
            ROS_ERROR("[Astar] boundary points out of map.");
            return make_pair(path, 0.0);
        }

        std::multimap<double, GridNodePtr> openSet;
        Eigen::Vector2i start_index;
        posToIndex(start, start_index);
        GridNodePtr start_point = grid_node_map[toAddress(start_index)];
        start_point->index = start_index;
        GridNodePtr currentPtr = nullptr;

        openSet.insert(make_pair(0.0, start_point));

        Eigen::Vector2i end_index;
        posToIndex(end, end_index);

        while ( !openSet.empty() )
        {
            auto iter  = std::begin(openSet);
            currentPtr = iter -> second;
            openSet.erase(iter);

            grid_node_map[toAddress(currentPtr->index)]->id = IN_CLOSE;
 
            if( currentPtr->index == end_index )
            {
                GridNode* p = currentPtr;
                double cost = p->gScore;
                Eigen::Vector2d p_world;
                while (p->parent != nullptr)
                {
                    indexToPos(p->index, p_world);
                    path.push_back(p_world);
                    p = p->parent;
                }
                indexToPos(p->index, p_world);
                path.push_back(p_world);

                reverse(path.begin(), path.end());

                return make_pair(path, cost);
            }

            Eigen::Vector2i neighbor_index;
            for(int i = -1; i <= 1; i++)
            {
                for(int j = -1; j <= 1; j++)
                {
                    if(i == 0 && j == 0) { continue; }
                    neighbor_index = currentPtr->index + Eigen::Vector2i(i ,j);

                    if(isInMap(neighbor_index))
                    {  
                        GridNodePtr neighborPtr = grid_node_map[toAddress(neighbor_index)];
                        if (neighborPtr->id == IS_UNKNOWN && !isOccupancy(neighbor_index))
                        {
                            double tg = ((i * j == 0) ? resolution : (resolution * 1.41)) + currentPtr -> gScore;
                            double heu = 1.0001 * (end_index - neighbor_index).norm() * resolution;

                            neighborPtr -> parent = currentPtr;
                            neighborPtr -> gScore = tg;
                            neighborPtr -> fScore = tg + heu;
                            neighborPtr -> index = neighbor_index;
                            neighborPtr -> id = IN_OPEN;
                            openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                        }
                        else if (neighborPtr->id == IN_OPEN)
                        {
                            double tg = ((i * j == 0) ? resolution : (resolution * 1.414)) + currentPtr -> gScore;
                            if (tg < neighborPtr->gScore)
                            {
                                double heu = 1.0001 * (end_index - neighbor_index).norm() * resolution;
                                neighborPtr -> parent = currentPtr;
                                neighborPtr -> gScore = tg;
                                neighborPtr -> fScore = tg + heu;
                            }
                        }
                        else
                        {
                            double tg = ((i * j == 0) ? resolution : (resolution * 1.414)) + currentPtr -> gScore;
                            if(tg < neighborPtr -> gScore)
                            {
                                double heu = 1.0001 * (end_index - neighbor_index).norm() * resolution;
                                neighborPtr -> parent = currentPtr;
                                neighborPtr -> gScore = tg;
                                neighborPtr -> fScore = tg + heu;
                                
                                neighborPtr -> id = IN_OPEN;
                                openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                            }
                        }
                    }
                }
            }   
        }

        ROS_ERROR("[Astar] Fails!!!");
        path.clear();

        return make_pair(path, 0.0);
    }
}