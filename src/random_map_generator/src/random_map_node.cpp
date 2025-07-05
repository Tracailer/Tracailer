#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

using namespace std;

// pcl
vector<Eigen::Vector2d> centers;
pcl::PointCloud<pcl::PointXYZ> cloud_map;
pcl::KdTreeFLANN<pcl::PointXYZ> kd_tree;
pcl::PointXYZ sensor_pose;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

// random
default_random_engine eng;
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_theta;
uniform_real_distribution<double> rand_radius;

// ros
ros::Timer vis_timer;
ros::Timer sensor_timer;
ros::Subscriber odom_sub;
ros::Publisher local_map_pub;
ros::Publisher global_map_pub;
ros::Publisher mesh_map_pub;
ros::Publisher polygon_map_pub;
sensor_msgs::PointCloud2 global_msg;
visualization_msgs::Marker mesh_msg;
std_msgs::Float64MultiArray polygon_msg;

// params
bool has_odom = false;
bool has_map = false;
bool fix_generator = false;
vector<int> obs_num = {1, 1, 1};
double resolution = 0.1;
double size_x = 30.0;
double size_y = 30.0;
double min_width = 0.3;
double max_width = 0.8;
double min_obs_dis = 0.3;
double vis_rate = 10.0;
double sensor_rate = 10.0;
double sensor_range = 5.0;

// laser
constexpr int LINE_NUM = 128;
// constexpr int LINE_NUM = 1024;
double laser_res = 2.0 * M_PI / LINE_NUM;
Eigen::VectorXi idx_map = Eigen::VectorXi::Constant(LINE_NUM, -1);
Eigen::VectorXd dis_map = Eigen::VectorXd::Constant(LINE_NUM, 9999.0);

bool crossBoolean2(Eigen::Vector2d a, Eigen::Vector2d b)
{
    return (a(0)*b(1)-b(0)*a(1) > 0);
}

pcl::PointCloud<pcl::PointXYZ> fillConvexPolygon(vector<Eigen::Vector2d> poly_vs)
{
    pcl::PointCloud<pcl::PointXYZ> cloud_polygon;

    if (poly_vs.size() < 3)
        return cloud_polygon;
    
    double down = 9999.0;
    double up = -9999.0;
    double left = 9999.0;
    double right = -9999.0;
    
    // AABB box
    for (size_t i=0; i<poly_vs.size(); i++)
    {
        if (poly_vs[i][0] > right)
            right = poly_vs[i][0];
        if (poly_vs[i][0] < left)
            left = poly_vs[i][0];
        if (poly_vs[i][1] > up)
            up = poly_vs[i][1];
        if (poly_vs[i][1] < down)
            down = poly_vs[i][1];
    }

    for (double x=left; x<right+resolution; x+=resolution)
    {
        for (double y=down; y<up+resolution; y+=resolution)
        {
            bool in_poly = false;
            Eigen::Vector2d O(x, y);

            for (size_t i=0; i<poly_vs.size() - 2; i++)
            {
                // if a point is in triangle
                Eigen::Vector2d A = poly_vs[0];
                Eigen::Vector2d B = poly_vs[i+1];
                Eigen::Vector2d C = poly_vs[i+2];
                if (crossBoolean2(B-A, O-A) && \
                    crossBoolean2(C-B, O-B) && \
                    crossBoolean2(A-C, O-C) )
                {
                    in_poly = true;
                    break;
                }                
            }

            if (in_poly)
            {
                pcl::PointXYZ pt;
                pt.x = x;
                pt.y = y;
                pt.z = 0.0;
                cloud_polygon.push_back(pt);
            }
        }
    }
    
    return cloud_polygon;
}

pair<vector<Eigen::Vector2d>, pcl::PointCloud<pcl::PointXYZ>> generatePolygon(int K)
{
    pcl::PointCloud<pcl::PointXYZ> cloud_polygon;

    rand_w = uniform_real_distribution<double>(min_width, max_width);
    rand_theta = uniform_real_distribution<double>(-M_PI, M_PI);

    double radius = rand_w(eng);
    double theta = rand_theta(eng);
    double angle_res = 2.0 * M_PI / K;
    double small_r = radius * sin(angle_res/2.0);

    rand_radius = uniform_real_distribution<double>(-small_r, small_r);

    vector<Eigen::Vector2d> vs;
    for (int i=0; i<K; i++)
    {
        double a = angle_res * i + theta;
        double delta_theta = rand_theta(eng);
        double delta_radius = rand_radius(eng);
        Eigen::Vector2d p(cos(a)*radius + cos(a+delta_theta)*delta_radius, \
                          sin(a)*radius + sin(a+delta_theta)*delta_radius);
        vs.push_back(p);
    }
    cloud_polygon = fillConvexPolygon(vs);

    return std::make_pair(vs, cloud_polygon);
} 

void generateRandomCase()
{
    pcl::PointXYZ pt_random;

    rand_x = uniform_real_distribution<double>(-size_x / 2.0, size_x / 2.0);
    rand_y = uniform_real_distribution<double>(-size_y / 2.0, size_y / 2.0);

    // generate polygon obs
    centers.clear();
    for (size_t k = 0; k<obs_num.size(); k++)
    {
        for (int j = 0; j < obs_num[k]; j++) 
        {
            double x, y;
            x = rand_x(eng);
            y = rand_y(eng);

            // if (sqrt(pow(x, 2) + pow(y, 2)) < 1.0) 
            // {
            //     i--;
            //     continue;
            // }

            x = floor(x / resolution) * resolution + resolution / 2.0;
            y = floor(y / resolution) * resolution + resolution / 2.0;
            
            bool collision = false;
            for (size_t i=0; i<centers.size(); i++)
            {
                if ((Eigen::Vector2d(x, y)-centers[i]).squaredNorm() < min_obs_dis * min_obs_dis)
                {
                    collision = true;
                    break;
                }
            }
            if (collision)
            {
                j--;
                continue;
            }
            else
            {
                centers.push_back(Eigen::Vector2d(x, y));
            }

            pair<vector<Eigen::Vector2d>, pcl::PointCloud<pcl::PointXYZ>> cloud_polygon = generatePolygon(k+3);
            for (size_t i=0; i<cloud_polygon.second.points.size(); i++)
            {
                pt_random.x = cloud_polygon.second.points[i].x + x;
                pt_random.y = cloud_polygon.second.points[i].y + y;
                pt_random.z = 0.0;
                cloud_map.points.push_back(pt_random);
            }

            vector<Eigen::Vector2d> vector_polygon = cloud_polygon.first;
            geometry_msgs::Point init_p;
            init_p.x = vector_polygon[0].x() + x;
            init_p.y = vector_polygon[0].y() + y;
            init_p.z = 0.0;
            polygon_msg.data.push_back(k+3);
            polygon_msg.data.push_back(init_p.x);
            polygon_msg.data.push_back(init_p.y);
            for (size_t i=1; i<k+2; i++)
            {
                mesh_msg.points.push_back(init_p);
                geometry_msgs::Point p;
                p.x = vector_polygon[i].x() + x;
                p.y = vector_polygon[i].y() + y;
                p.z = 0.0;
                mesh_msg.points.push_back(p);
                polygon_msg.data.push_back(p.x);
                polygon_msg.data.push_back(p.y);
                p.x = vector_polygon[i+1].x() + x;
                p.y = vector_polygon[i+1].y() + y;
                p.z = 0.0;
                mesh_msg.points.push_back(p);
            }
            polygon_msg.data.push_back(vector_polygon.back().x()+x);
            polygon_msg.data.push_back(vector_polygon.back().y()+y);
        }
    }

    cloud_map.width = cloud_map.points.size();
    cloud_map.height = 1;
    cloud_map.is_dense = true;
    has_map = true;

    pcl::toROSMsg(cloud_map, global_msg);
    global_msg.header.frame_id = "world";

 	mesh_msg.id = 0;
 	mesh_msg.type = visualization_msgs::Marker::TRIANGLE_LIST;
 	mesh_msg.action = visualization_msgs::Marker::ADD;
    mesh_msg.pose.orientation.w = 1.0;
 	mesh_msg.scale.x = 1.0;
 	mesh_msg.scale.y = 1.0;
 	mesh_msg.scale.z = 1.0;
 	mesh_msg.color.r = 0.2;
 	mesh_msg.color.g = 0.2;
 	mesh_msg.color.b = 0.2;
 	mesh_msg.color.a = 0.3;
    mesh_msg.header.frame_id = "world";
}

void generateNarrowCase()
{
    pcl::PointXYZ pt_random;

    vector<double> px;
    vector<double> py;
    px.push_back(0.0);
    px.push_back(0.0);
    py.push_back(3.0);
    py.push_back(-3.0);

    for (size_t p=0; p<px.size(); p++)
    {
        double x = px[p];
        double y = py[p];

        // generate polygon obs
        x = floor(x / resolution) * resolution + resolution / 2.0;
        y = floor(y / resolution) * resolution + resolution / 2.0;
        
        pair<vector<Eigen::Vector2d>, pcl::PointCloud<pcl::PointXYZ>> cloud_polygon;
        vector<Eigen::Vector2d> vs;
        double w = 2;
        vs.push_back(Eigen::Vector2d(-3, -w));
        vs.push_back(Eigen::Vector2d(3, -w));
        vs.push_back(Eigen::Vector2d(3, w));
        vs.push_back(Eigen::Vector2d(-3, w));
        cloud_polygon.first = vs;
        cloud_polygon.second = fillConvexPolygon(vs);

        for (size_t i=0; i<cloud_polygon.second.points.size(); i++)
        {
            pt_random.x = cloud_polygon.second.points[i].x + x;
            pt_random.y = cloud_polygon.second.points[i].y + y;
            pt_random.z = 0.0;
            cloud_map.points.push_back(pt_random);
        }

        vector<Eigen::Vector2d> vector_polygon = cloud_polygon.first;
        geometry_msgs::Point init_p;
        init_p.x = vector_polygon[0].x() + x;
        init_p.y = vector_polygon[0].y() + y;
        init_p.z = 0.0;
        polygon_msg.data.push_back(4);
        polygon_msg.data.push_back(init_p.x);
        polygon_msg.data.push_back(init_p.y);
        for (int i=1; i<3; i++)
        {
            mesh_msg.points.push_back(init_p);
            geometry_msgs::Point p;
            p.x = vector_polygon[i].x() + x;
            p.y = vector_polygon[i].y() + y;
            p.z = 0.0;
            mesh_msg.points.push_back(p);
            polygon_msg.data.push_back(p.x);
            polygon_msg.data.push_back(p.y);
            p.x = vector_polygon[i+1].x() + x;
            p.y = vector_polygon[i+1].y() + y;
            p.z = 0.0;
            mesh_msg.points.push_back(p);
        }
        polygon_msg.data.push_back(vector_polygon.back().x()+x);
        polygon_msg.data.push_back(vector_polygon.back().y()+y);
    }

    cloud_map.width = cloud_map.points.size();
    cloud_map.height = 1;
    cloud_map.is_dense = true;
    has_map = true;

    pcl::toROSMsg(cloud_map, global_msg);
    global_msg.header.frame_id = "world";

 	mesh_msg.id = 0;
 	mesh_msg.type = visualization_msgs::Marker::TRIANGLE_LIST;
 	mesh_msg.action = visualization_msgs::Marker::ADD;
    mesh_msg.pose.orientation.w = 1.0;
 	mesh_msg.scale.x = 1.0;
 	mesh_msg.scale.y = 1.0;
 	mesh_msg.scale.z = 1.0;
 	mesh_msg.color.r = 0.2;
 	mesh_msg.color.g = 0.2;
 	mesh_msg.color.b = 0.2;
 	mesh_msg.color.a = 0.3;
    mesh_msg.header.frame_id = "world";
}

void generateComplexCase()
{
    pcl::PointXYZ pt_random;

    vector<double> px{0.0, 0.0, -4.0, -6.0};
    vector<double> py{-3.0, 3.0, -3.0, 0.0};
    double w = 2.5;
    // double w = 2.6;
    vector<Eigen::Vector2d> a1{{-3.0, -w}, {3.0, -w}, {3.0, w}, {-3.0, w}};
    vector<Eigen::Vector2d> a2{{-3.0, -w}, {3.0, -w}, {3.0, w}, {-3.0, w}};
    vector<Eigen::Vector2d> a3{{-1.0, -2.0}, {1.0, -2.0}, {1.0, 2.0}, {-1.0, 2.0}};
    vector<Eigen::Vector2d> a4{{-1.0, -5.0}, {1.0, -5.0}, {1.0, 5.0}, {-1.0, 5.0}};
    vector<vector<Eigen::Vector2d>> vs_list{a1, a2, a3, a4};

    for (size_t p=0; p<px.size(); p++)
    {
        double x = px[p];
        double y = py[p];

        // generate polygon obs
        x = floor(x / resolution) * resolution + resolution / 2.0;
        y = floor(y / resolution) * resolution + resolution / 2.0;
        
        pair<vector<Eigen::Vector2d>, pcl::PointCloud<pcl::PointXYZ>> cloud_polygon;
        cloud_polygon.first = vs_list[p];
        cloud_polygon.second = fillConvexPolygon(vs_list[p]);

        for (size_t i=0; i<cloud_polygon.second.points.size(); i++)
        {
            pt_random.x = cloud_polygon.second.points[i].x + x;
            pt_random.y = cloud_polygon.second.points[i].y + y;
            pt_random.z = 0.0;
            cloud_map.points.push_back(pt_random);
        }

        vector<Eigen::Vector2d> vector_polygon = cloud_polygon.first;
        geometry_msgs::Point init_p;
        init_p.x = vector_polygon[0].x() + x;
        init_p.y = vector_polygon[0].y() + y;
        init_p.z = 0.0;
        polygon_msg.data.push_back(4);
        polygon_msg.data.push_back(init_p.x);
        polygon_msg.data.push_back(init_p.y);
        for (int i=1; i<3; i++)
        {
            mesh_msg.points.push_back(init_p);
            geometry_msgs::Point p;
            p.x = vector_polygon[i].x() + x;
            p.y = vector_polygon[i].y() + y;
            p.z = 0.0;
            mesh_msg.points.push_back(p);
            polygon_msg.data.push_back(p.x);
            polygon_msg.data.push_back(p.y);
            p.x = vector_polygon[i+1].x() + x;
            p.y = vector_polygon[i+1].y() + y;
            p.z = 0.0;
            mesh_msg.points.push_back(p);
        }
        polygon_msg.data.push_back(vector_polygon.back().x()+x);
        polygon_msg.data.push_back(vector_polygon.back().y()+y);
    }

    cloud_map.width = cloud_map.points.size();
    cloud_map.height = 1;
    cloud_map.is_dense = true;
    has_map = true;

    pcl::toROSMsg(cloud_map, global_msg);
    global_msg.header.frame_id = "world";

 	mesh_msg.id = 0;
 	mesh_msg.type = visualization_msgs::Marker::TRIANGLE_LIST;
 	mesh_msg.action = visualization_msgs::Marker::ADD;
    mesh_msg.pose.orientation.w = 1.0;
 	mesh_msg.scale.x = 1.0;
 	mesh_msg.scale.y = 1.0;
 	mesh_msg.scale.z = 1.0;
 	mesh_msg.color.r = 0.2;
 	mesh_msg.color.g = 0.2;
 	mesh_msg.color.b = 0.2;
 	mesh_msg.color.a = 0.3;
    mesh_msg.header.frame_id = "world";
}

void generateThinGapCase()
{
    pcl::PointXYZ pt_random;

    vector<double> px{15.0, -15.0, 0.0, 0.0};
    vector<double> py{0.0, 0.0, 15.0, -15.0};
    double w = 0.05;
    // double w = 2.6;
    double gap_width_half = 0.4;
    vector<Eigen::Vector2d> wall1{{-0.2, -15.0}, {0.2, -15.0}, {0.2, 15.0}, {-0.2, 15.0}};
    vector<Eigen::Vector2d> wall2{{-15.0, -0.2}, {15.0, -0.2}, {15.0, 0.2}, {-15.0, 0.2}};
    vector<vector<Eigen::Vector2d>> vs_list{wall1, wall1, wall2, wall2};
    vector<Eigen::Vector2d> pgap{{10.0, -10.0}, 
                                {-8.0, -6.0}, 
                                {-8.0, -2.0},
                                {2.0, 2.0},
                                {6.0, 4.0},
                                {-2.0, 7.0},
                                {3.0, 10.0}, 
                                {2.0, 14.0},};
    for (size_t i=0; i<pgap.size(); i++)
    {
        double h = (pgap[i].x() + 15.0 - gap_width_half)/2.0;
        vector<Eigen::Vector2d> gap1{{-h, -w}, {h, -w}, {h, w}, {-h, w}};
        vs_list.push_back(gap1);
        px.push_back(pgap[i].x() - h - gap_width_half);
        py.push_back(pgap[i].y());
        h = (15.0 - pgap[i].x() - gap_width_half)/2.0;
        vector<Eigen::Vector2d> gap2{{-h, -w}, {h, -w}, {h, w}, {-h, w}};
        vs_list.push_back(gap2);
        px.push_back(pgap[i].x() + h + gap_width_half);
        py.push_back(pgap[i].y());
    }

    for (size_t p=0; p<px.size(); p++)
    {
        double x = px[p];
        double y = py[p];

        // generate polygon obs
        x = floor(x / resolution) * resolution + resolution / 2.0;
        y = floor(y / resolution) * resolution + resolution / 2.0;
        
        pair<vector<Eigen::Vector2d>, pcl::PointCloud<pcl::PointXYZ>> cloud_polygon;
        cloud_polygon.first = vs_list[p];
        cloud_polygon.second = fillConvexPolygon(vs_list[p]);

        for (size_t i=0; i<cloud_polygon.second.points.size(); i++)
        {
            pt_random.x = cloud_polygon.second.points[i].x + x;
            pt_random.y = cloud_polygon.second.points[i].y + y;
            pt_random.z = 0.0;
            cloud_map.points.push_back(pt_random);
        }

        vector<Eigen::Vector2d> vector_polygon = cloud_polygon.first;
        geometry_msgs::Point init_p;
        init_p.x = vector_polygon[0].x() + x;
        init_p.y = vector_polygon[0].y() + y;
        init_p.z = 0.0;
        polygon_msg.data.push_back(4);
        polygon_msg.data.push_back(init_p.x);
        polygon_msg.data.push_back(init_p.y);
        for (int i=1; i<3; i++)
        {
            mesh_msg.points.push_back(init_p);
            geometry_msgs::Point p;
            p.x = vector_polygon[i].x() + x;
            p.y = vector_polygon[i].y() + y;
            p.z = 0.0;
            mesh_msg.points.push_back(p);
            polygon_msg.data.push_back(p.x);
            polygon_msg.data.push_back(p.y);
            p.x = vector_polygon[i+1].x() + x;
            p.y = vector_polygon[i+1].y() + y;
            p.z = 0.0;
            mesh_msg.points.push_back(p);
        }
        polygon_msg.data.push_back(vector_polygon.back().x()+x);
        polygon_msg.data.push_back(vector_polygon.back().y()+y);
    }

    cloud_map.width = cloud_map.points.size();
    cloud_map.height = 1;
    cloud_map.is_dense = true;
    has_map = true;

    pcl::toROSMsg(cloud_map, global_msg);
    global_msg.header.frame_id = "world";

 	mesh_msg.id = 0;
 	mesh_msg.type = visualization_msgs::Marker::TRIANGLE_LIST;
 	mesh_msg.action = visualization_msgs::Marker::ADD;
    mesh_msg.pose.orientation.w = 1.0;
 	mesh_msg.scale.x = 1.0;
 	mesh_msg.scale.y = 1.0;
 	mesh_msg.scale.z = 1.0;
 	mesh_msg.color.r = 0.2;
 	mesh_msg.color.g = 0.2;
 	mesh_msg.color.b = 0.2;
 	mesh_msg.color.a = 0.3;
    mesh_msg.header.frame_id = "world";
}

void generatePointCase()
{
    pcl::PointXYZ pt_random;

    double w = 0.01;
    double h = 3.0;
    double gap = 0.7;
    double x_cord = 2.0;
    vector<double> px{x_cord, x_cord};
    vector<double> py{h-0.5, h-0.5};
    py[1] = py[1] - 2*h - gap;
    vector<Eigen::Vector2d> point{{-w, -h}, {w, -h}, {w, h}, {-w, h}};
    vector<vector<Eigen::Vector2d>> vs_list({point, point});

    double h_temp[2] = {h+0.5, h};
    for (int i=0; i<2; i++)
    {
        x_cord += 3.0;
        double y = h_temp[i];
        px.push_back(x_cord);
        px.push_back(x_cord);
        py.push_back(y);
        py.push_back(y-2*h-gap);
        vs_list.push_back(point);
        vs_list.push_back(point);
    }

    // for (size_t p=0; p<1; p++)
    for (size_t p=0; p<px.size(); p++)
    {
        double x = px[p];
        double y = py[p];

        // generate polygon obs
        x = floor(x / resolution) * resolution + resolution / 2.0;
        y = floor(y / resolution) * resolution + resolution / 2.0;
        
        pair<vector<Eigen::Vector2d>, pcl::PointCloud<pcl::PointXYZ>> cloud_polygon;
        cloud_polygon.first = vs_list[p];
        cloud_polygon.second = fillConvexPolygon(vs_list[p]);

        for (size_t i=0; i<cloud_polygon.second.points.size(); i++)
        {
            pt_random.x = cloud_polygon.second.points[i].x + x;
            pt_random.y = cloud_polygon.second.points[i].y + y;
            pt_random.z = 0.0;
            cloud_map.points.push_back(pt_random);
        }

        vector<Eigen::Vector2d> vector_polygon = cloud_polygon.first;
        geometry_msgs::Point init_p;
        init_p.x = vector_polygon[0].x() + x;
        init_p.y = vector_polygon[0].y() + y;
        init_p.z = 0.0;
        polygon_msg.data.push_back(4);
        polygon_msg.data.push_back(init_p.x);
        polygon_msg.data.push_back(init_p.y);
        for (int i=1; i<3; i++)
        {
            mesh_msg.points.push_back(init_p);
            geometry_msgs::Point p;
            p.x = vector_polygon[i].x() + x;
            p.y = vector_polygon[i].y() + y;
            p.z = 0.0;
            mesh_msg.points.push_back(p);
            polygon_msg.data.push_back(p.x);
            polygon_msg.data.push_back(p.y);
            p.x = vector_polygon[i+1].x() + x;
            p.y = vector_polygon[i+1].y() + y;
            p.z = 0.0;
            mesh_msg.points.push_back(p);
        }
        polygon_msg.data.push_back(vector_polygon.back().x()+x);
        polygon_msg.data.push_back(vector_polygon.back().y()+y);
    }

    cloud_map.width = cloud_map.points.size();
    cloud_map.height = 1;
    cloud_map.is_dense = true;
    has_map = true;

    pcl::toROSMsg(cloud_map, global_msg);
    global_msg.header.frame_id = "world";

 	mesh_msg.id = 0;
 	mesh_msg.type = visualization_msgs::Marker::TRIANGLE_LIST;
 	mesh_msg.action = visualization_msgs::Marker::ADD;
    mesh_msg.pose.orientation.w = 1.0;
 	mesh_msg.scale.x = 1.0;
 	mesh_msg.scale.y = 1.0;
 	mesh_msg.scale.z = 1.0;
 	mesh_msg.color.r = 0.2;
 	mesh_msg.color.g = 0.2;
 	mesh_msg.color.b = 0.2;
 	mesh_msg.color.a = 0.3;
    mesh_msg.header.frame_id = "world";
}

void generateParkCase()
{
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(ros::package::getPath("random_map_generator")\
        +"/meshes/park.pcd", cloud_map) == -1)
    {
        PCL_ERROR("Failed to read PCD file.\n");
        return;
    }

    for (size_t i=0; i<cloud_map.points.size(); i++)
    {
        double temp = cloud_map.points[i].z;
        if (temp < 1e-3)
        {
            cloud_map.points[i].x = 100.0;
            cloud_map.points[i].y = 100.0;
            cloud_map.points[i].z = 100.0;
            continue;
        }
    }

    cloud_map.width = cloud_map.points.size();
    cloud_map.height = 1;
    cloud_map.is_dense = true;
    has_map = true;

    pcl::toROSMsg(cloud_map, global_msg);
    global_msg.header.frame_id = "world";

 	mesh_msg.id = 0;
 	mesh_msg.type = visualization_msgs::Marker::MESH_RESOURCE;
 	mesh_msg.action = visualization_msgs::Marker::ADD;
    mesh_msg.pose.orientation.w = 1.0;
 	mesh_msg.scale.x = 1.0;
 	mesh_msg.scale.y = 1.0;
 	mesh_msg.scale.z = 1.0;
 	mesh_msg.color.a = 0.5;
 	mesh_msg.color.r = 0.0;
 	mesh_msg.color.g = 0.0;
 	mesh_msg.color.b = 0.0;
    mesh_msg.header.frame_id = "world";
    mesh_msg.mesh_resource = "package://random_map_generator/meshes/park.dae";
    return;
}

void visCallback(const ros::TimerEvent &e)
{
    if (!has_map)
        return;
    
    global_map_pub.publish(global_msg);
    mesh_map_pub.publish(mesh_msg);
    polygon_map_pub.publish(polygon_msg);
}

void rcvOdomCallBack(const nav_msgs::OdometryConstPtr msg)
{
    sensor_pose.x = msg->pose.pose.position.x;
    sensor_pose.y = msg->pose.pose.position.y;
    sensor_pose.z = 0.0;
    has_odom = true;
}

void sensorCallback(const ros::TimerEvent &e)
{
    if (!has_map || !has_odom)
        return;
    static int count = 0;
    count++;
    
    pcl::PointCloud<pcl::PointXYZ> local_map;

    pointIdxRadiusSearch.clear();
    pointRadiusSquaredDistance.clear();
    idx_map.setConstant(-1);
    dis_map.setConstant(9999.0);

    pcl::PointXYZ pt;
    if (kd_tree.radiusSearch(sensor_pose, sensor_range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) 
    {
        for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++) 
        {
            pt = cloud_map.points[pointIdxRadiusSearch[i]];
            int idx = floor((atan2(pt.y - sensor_pose.y, pt.x - sensor_pose.x) + M_PI + laser_res / 2.0) / laser_res);
            if (idx >= 0 && idx < LINE_NUM && dis_map[idx] > pointRadiusSquaredDistance[i])
            {
                idx_map[idx] = idx;
                dis_map[idx] = pointRadiusSquaredDistance[i];
            }
        }

        for (int i=0; i<LINE_NUM; i++)
        {
            if (idx_map[i] != -1)
            {
                double angle = idx_map[i] * laser_res - M_PI;
                double dist = sqrt(dis_map[i]);
                pt.x = dist*cos(angle) + sensor_pose.x;
                pt.y = dist*sin(angle) + sensor_pose.y;
                pt.z = 0.0;
                local_map.push_back(pt);
            }
        }
    }    

    local_map.width = local_map.points.size();
    local_map.height = 1;
    local_map.is_dense = true;

    sensor_msgs::PointCloud2 local_msg;
    pcl::toROSMsg(local_map, local_msg);
    local_msg.header.frame_id = "world";
    local_map_pub.publish(local_msg);
}

int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "random_map_node");
    ros::NodeHandle nh("~");

    int case_id = 0;
    nh.param<std::vector<int>>("map/obs_num", obs_num, std::vector<int>());
	nh.getParam("map/resolution", resolution);
	nh.getParam("map/fix_generator", fix_generator);
	nh.getParam("map/size_x", size_x);
	nh.getParam("map/size_y", size_y);
	nh.getParam("map/min_width", min_width);
	nh.getParam("map/max_width", max_width);
	nh.getParam("map/min_obs_dis", min_obs_dis);
	nh.getParam("map/vis_rate", vis_rate);
	nh.getParam("map/sensor_rate", sensor_rate);
	nh.getParam("map/sensor_range", sensor_range);
	nh.getParam("map/case_id", case_id);

    if (!fix_generator)
    {
        random_device rd;
        eng = default_random_engine(rd());
    }
    else
        eng = default_random_engine(0);
	  
    odom_sub  = nh.subscribe("odom", 1000, rcvOdomCallBack);
    local_map_pub = nh.advertise<sensor_msgs::PointCloud2>("local_cloud", 1);
    global_map_pub = nh.advertise<sensor_msgs::PointCloud2>("global_cloud", 1);
    mesh_map_pub = nh.advertise<visualization_msgs::Marker>("mesh_obstacles", 1);
    polygon_map_pub = nh.advertise<std_msgs::Float64MultiArray>("global_polygon", 1);

    switch (case_id)
    {
        case 0:
            generateRandomCase();
            break;
        case 1:
            generateNarrowCase();
            break;
        case 2:
            generateComplexCase();
            break;
        case 3:
            generateThinGapCase();
            break;
        case 4:
            generatePointCase();
            break;
        case 5:
            generateParkCase();
            break;
        default:
            generateRandomCase();
            break;
    }

    kd_tree.setInputCloud(cloud_map.makeShared());

    vis_timer = nh.createTimer(ros::Duration(1.0/vis_rate), visCallback);
    sensor_timer = nh.createTimer(ros::Duration(1.0/sensor_rate), sensorCallback);

	ros::spin();

    return 0;
}