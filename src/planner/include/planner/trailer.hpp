#pragma once

#include <eigen3/Eigen/Eigen>
#include <chrono>
#include <random>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define PI_X_2 6.283185307179586
#define PRINTF_WHITE(STRING) std::cout<<STRING
#define PRINT_GREEN(STRING) std::cout<<"\033[92m"<<STRING<<"\033[m\n"
#define PRINT_RED(STRING) std::cout<<"\033[31m"<<STRING<<"\033[m\n"
#define PRINT_YELLOW(STRING) std::cout<<"\033[33m"<<STRING<<"\033[m\n"

using namespace std;

namespace trailer_planner
{
    class Trailer
    {
        public:
            double wheel_base;
            double width;
            double rear_length;
            double max_steer;
            double max_dtheta;
            double terminal_tol;
            double box_length;
            double box_width;
            vector<double> length;
            vector<double> Ltail;
            vector<double> Lhead;
            vector<double> pthetas;
            Eigen::MatrixXd terminal_area;
            Eigen::Matrix2Xd terminal_points;
            visualization_msgs::Marker mesh_msg;
            visualization_msgs::MarkerArray array_msg;
            visualization_msgs::Marker tractor_msg;
            visualization_msgs::Marker trailer_msg;

            ros::Publisher mesh_pub;
            ros::Publisher t0_pub, t1_pub, t2_pub, t3_pub;
            ros::Publisher terminal_pub;
            
        public:
            inline void init(ros::NodeHandle& nh)
            {
                nh.getParam("/trailer/wheel_base", wheel_base);
                nh.getParam("/trailer/width", width);
                nh.getParam("/trailer/rear_length", rear_length);
                nh.getParam("/trailer/max_steer", max_steer);
                nh.getParam("/trailer/max_dtheta", max_dtheta);
                nh.getParam("/trailer/terminal_tol", terminal_tol);
                nh.param<std::vector<double>>("/trailer/length", length, std::vector<double>());
                nh.param<std::vector<double>>("/trailer/Ltail", Ltail, std::vector<double>());
                nh.param<std::vector<double>>("/trailer/Lhead", Lhead, std::vector<double>());
                nh.param<std::vector<double>>("/trailer/init_pthetas", pthetas, std::vector<double>());

                if (Lhead.size() > TRAILER_NUM)
                {
                    int delte_num = Lhead.size() - TRAILER_NUM;
                    for (int i=0; i<delte_num; i++)
                    {
                        length.pop_back();
                        Lhead.pop_back();
                        Ltail.pop_back();
                        pthetas.pop_back();
                    }
                }

                mesh_pub = nh.advertise<visualization_msgs::MarkerArray>("trailer/mesh", 1);
                terminal_pub = nh.advertise<visualization_msgs::Marker>("trailer/terminal", 1);
                t0_pub = nh.advertise<visualization_msgs::Marker>("trailer/t0", 1);
                t1_pub = nh.advertise<visualization_msgs::Marker>("trailer/t1", 1);
                t2_pub = nh.advertise<visualization_msgs::Marker>("trailer/t2", 1);
                t3_pub = nh.advertise<visualization_msgs::Marker>("trailer/t3", 1);

                assert(TRAILER_NUM==Lhead.size());
                assert(TRAILER_NUM==Ltail.size());
                assert(TRAILER_NUM==pthetas.size()-3);

                box_length = length[0] - rear_length + length[length.size()-1] / 2.0 + terminal_tol;
                for (size_t i=0; i<Lhead.size(); i++)
                {
                    box_length += Lhead[i];
                    box_length += Ltail[i];
                }
                box_width = width + terminal_tol;

                mesh_msg.id = 0;
                mesh_msg.type = visualization_msgs::Marker::LINE_LIST;
                mesh_msg.action = visualization_msgs::Marker::ADD;
                mesh_msg.pose.orientation.w = 1.0;
                mesh_msg.scale.x = 0.03;
                mesh_msg.color.r = 1.0;
                mesh_msg.color.g = 0.0;
                mesh_msg.color.b = 1.0;
                mesh_msg.color.a = 1.0;
                mesh_msg.header.frame_id = "world";

                // box0
                geometry_msgs::Point p;
                p.z = 0.0;
                p.x = -rear_length;
                p.y = width * 0.5;
                mesh_msg.points.push_back(p);
                p.x += length[0]; 
                mesh_msg.points.push_back(p);
                mesh_msg.points.push_back(p);
                p.y -= width;
                mesh_msg.points.push_back(p);
                mesh_msg.points.push_back(p);
                p.x -= length[0];
                mesh_msg.points.push_back(p);
                mesh_msg.points.push_back(p);
                p.y += width;
                mesh_msg.points.push_back(p);

                // left wheel
                p.x += rear_length * 0.5;
                p.y -= rear_length * 0.2;
                // p.y += rear_length * 0.5;
                mesh_msg.points.push_back(p);
                p.x += rear_length;
                mesh_msg.points.push_back(p);
                mesh_msg.points.push_back(p);
                p.y -= rear_length;
                mesh_msg.points.push_back(p);
                mesh_msg.points.push_back(p);
                p.x -= rear_length;
                mesh_msg.points.push_back(p);
                mesh_msg.points.push_back(p);
                p.y += rear_length;
                mesh_msg.points.push_back(p);

                // right wheel
                p.y = p.y - width + 1.3 * rear_length;
                // p.y -= width;
                mesh_msg.points.push_back(p);
                p.x += rear_length;
                mesh_msg.points.push_back(p);
                mesh_msg.points.push_back(p);
                p.y -= rear_length;
                mesh_msg.points.push_back(p);
                mesh_msg.points.push_back(p);
                p.x -= rear_length;
                mesh_msg.points.push_back(p);
                mesh_msg.points.push_back(p);
                p.y += rear_length;
                mesh_msg.points.push_back(p);

                // front wheel
                p.x = length[0] - rear_length * 2.2;
                // p.x = length[0] - rear_length * 1.5;
                p.y = rear_length * 0.5;
                mesh_msg.points.push_back(p);
                p.x += rear_length;
                mesh_msg.points.push_back(p);
                mesh_msg.points.push_back(p);
                p.y -= rear_length;
                mesh_msg.points.push_back(p);
                mesh_msg.points.push_back(p);
                p.x -= rear_length;
                mesh_msg.points.push_back(p);
                mesh_msg.points.push_back(p);
                p.y += rear_length;
                mesh_msg.points.push_back(p);

                // tail0
                p.x = p.y = 0.0;
                if (Ltail[0]>1e-4)
                {
                    mesh_msg.points.push_back(p);
                    p.x -= Ltail[0];
                    mesh_msg.points.push_back(p);
                }
                array_msg.markers.push_back(mesh_msg);

                for (size_t i=0; i<TRAILER_NUM; i++)
                {
                    mesh_msg.id++;
                    mesh_msg.color.r = 1.0 * (rand() % 1000) / 1000.0;
                    mesh_msg.color.g = 1.0 * (rand() % 1000) / 1000.0;
                    mesh_msg.color.b = 1.0 * (rand() % 1000) / 1000.0;
                    mesh_msg.points.clear();

                    // head_i
                    p.x = p.y = 0.0;
                    mesh_msg.points.push_back(p);
                    p.x -= Lhead[i];
                    mesh_msg.points.push_back(p);

                    // box_{i+1}
                    p.x -= 0.5 * length[i+1];
                    p.y += 0.5 * width;
                    mesh_msg.points.push_back(p);
                    p.x += length[i+1];
                    mesh_msg.points.push_back(p);
                    mesh_msg.points.push_back(p);
                    p.y -= width;
                    mesh_msg.points.push_back(p);
                    mesh_msg.points.push_back(p);
                    p.x -= length[i+1];
                    mesh_msg.points.push_back(p);
                    mesh_msg.points.push_back(p);
                    p.y += width;
                    mesh_msg.points.push_back(p);
                    
                    // tail_{i+1}
                    p.y = 0.0;
                    p.x += 0.5 * length[i+1];
                    if (i<TRAILER_NUM-1 && Ltail[i+1]>1e-4)
                    {
                        mesh_msg.points.push_back(p);
                        p.x -= Ltail[i+1];
                        mesh_msg.points.push_back(p);
                        p.x += Ltail[i+1];
                    }

                    // left_wheel
                    p.x -= 0.5 * rear_length;
                    p.y += 0.5 * width - 0.2 * rear_length; 
                    // p.y += 0.5 * width + 0.5 * rear_length; 
                    mesh_msg.points.push_back(p);
                    p.x += rear_length;
                    mesh_msg.points.push_back(p);
                    mesh_msg.points.push_back(p);
                    p.y -= rear_length;
                    mesh_msg.points.push_back(p);
                    mesh_msg.points.push_back(p);
                    p.x -= rear_length;
                    mesh_msg.points.push_back(p);
                    mesh_msg.points.push_back(p);
                    p.y += rear_length;
                    mesh_msg.points.push_back(p);

                    // right_wheel
                    p.y = p.y - width + 1.3 * rear_length;
                    // p.y -= width;
                    mesh_msg.points.push_back(p);
                    p.x += rear_length;
                    mesh_msg.points.push_back(p);
                    mesh_msg.points.push_back(p);
                    p.y -= rear_length;
                    mesh_msg.points.push_back(p);
                    mesh_msg.points.push_back(p);
                    p.x -= rear_length;
                    mesh_msg.points.push_back(p);
                    mesh_msg.points.push_back(p);
                    p.y += rear_length;
                    mesh_msg.points.push_back(p);

                    array_msg.markers.push_back(mesh_msg);
                }

                // diao
                tractor_msg.header.frame_id = "world";
                tractor_msg.id = 100;
                tractor_msg.type = visualization_msgs::Marker::MESH_RESOURCE;
                tractor_msg.action = visualization_msgs::Marker::ADD;
                tractor_msg.pose.position.x = 0.0;
                tractor_msg.pose.position.y = 0.0;
                tractor_msg.pose.position.z = 0.0;
                tractor_msg.pose.orientation.w = 1.0;
                tractor_msg.mesh_use_embedded_materials = true;
                tractor_msg.color.a = 0.0;
                tractor_msg.color.r = 0.0;
                tractor_msg.color.g = 0.0;
                tractor_msg.color.b = 0.0;
                tractor_msg.scale.x = 0.2;
                tractor_msg.scale.y = 0.4;
                tractor_msg.scale.z = 0.2;
                tractor_msg.mesh_resource = "package://planner/meshes/car0.dae";
                trailer_msg = tractor_msg;
                trailer_msg.mesh_resource = "package://planner/meshes/trailer.dae";
                trailer_msg.scale.x = 1.0;
                trailer_msg.scale.y = 1.0;
                trailer_msg.scale.z = 1.0;
            }

            inline void showTrailer(const Eigen::VectorXd& state, int id)
            {
                assert((size_t)state.size()==3+TRAILER_NUM);

                visualization_msgs::MarkerArray msg(array_msg);
                for (size_t i=0; i<msg.markers.size(); i++)
                {
                    msg.markers[i].id += id;
                    if (i == 0)
                    {
                        msg.markers[0].pose.position.x = state[0];
                        msg.markers[0].pose.position.y = state[1];
                    }
                    else if (i==1)
                    {
                        msg.markers[i].pose.position.x = state[0] - Ltail[0] * cos(state[2]);
                        msg.markers[i].pose.position.y = state[1] - Ltail[0] * sin(state[2]);
                    }
                    else
                    {
                        msg.markers[i].pose.position.x = msg.markers[i-1].pose.position.x
                                                                - (Ltail[i-1] + Lhead[i-2]) * cos(state[i+1]);
                        msg.markers[i].pose.position.y = msg.markers[i-1].pose.position.y
                                                                - (Ltail[i-1] + Lhead[i-2]) * sin(state[i+1]);
                    }
                    msg.markers[i].pose.orientation.w = cos(state[i+2]/2.0);
                    msg.markers[i].pose.orientation.x = 0.0;
                    msg.markers[i].pose.orientation.y = 0.0;
                    msg.markers[i].pose.orientation.z = sin(state[i+2]/2.0);
                }

                // diao
                tractor_msg.pose.position.x = state[0];
                tractor_msg.pose.position.y = state[1];
                tractor_msg.pose.orientation.w = cos(state[2]/2.0);
                tractor_msg.pose.orientation.x = 0.0;
                tractor_msg.pose.orientation.y = 0.0;
                tractor_msg.pose.orientation.z = sin(state[2]/2.0);
                t0_pub.publish(tractor_msg);
                trailer_msg.pose.position.x = state[0] - Ltail[0] * cos(state[2]) - Lhead[0] * cos(state[3]);
                trailer_msg.pose.position.y = state[1] - Ltail[0] * sin(state[2]) - Lhead[0] * sin(state[3]);
                trailer_msg.pose.orientation.w = cos(state[3]/2.0);
                trailer_msg.pose.orientation.z = sin(state[3]/2.0);
                t1_pub.publish(trailer_msg);
                if (TRAILER_NUM > 1)
                {
                    trailer_msg.pose.position.x -= (Lhead[1] * cos(state[4]) + Ltail[1] * cos(state[3]));
                    trailer_msg.pose.position.y -= (Lhead[1] * sin(state[4]) + Ltail[1] * sin(state[3]));
                    trailer_msg.pose.orientation.w = cos(state[4]/2.0);
                    trailer_msg.pose.orientation.z = sin(state[4]/2.0);
                    t2_pub.publish(trailer_msg);
                }
                if (TRAILER_NUM > 2)
                {
                    trailer_msg.pose.position.x -= (Lhead[2] * cos(state(5)) + Ltail[2] * cos(state(4)));
                    trailer_msg.pose.position.y -= (Lhead[2] * sin(state(5)) + Ltail[2] * sin(state(4)));
                    trailer_msg.pose.orientation.w = cos(state(5)/2.0); 
                    trailer_msg.pose.orientation.z = sin(state(5)/2.0);
                    t3_pub.publish(trailer_msg);
                }
                
                mesh_pub.publish(msg);
            }

            inline void setShowTerminal(const Eigen::Vector3d& se2)
            {
                terminal_area.resize(4, 4);
                terminal_area.setZero();
                terminal_points.resize(2, 4);
                
                visualization_msgs::Marker msg;
                msg.id = 0;
                msg.type = visualization_msgs::Marker::LINE_LIST;
                msg.action = visualization_msgs::Marker::ADD;
                msg.pose.orientation.w = 1.0;
                msg.scale.x = 0.03;
                msg.color.r = msg.color.g = msg.color.b = 0.0;
                msg.color.a = 1.0;
                msg.header.frame_id = "world";

                double h = box_length / 2.0;
                double w = box_width / 2.0;
                double st = sin(se2(2));
                double ct = cos(se2(2));
                geometry_msgs::Point p;
                p.z = 0.0;
                p.x = se2.x() + ct*h - st*w;
                p.y = se2.y() + st*h + ct*w;
                msg.points.push_back(p);
                terminal_points.col(0) = Eigen::Vector2d(p.x, p.y);
                terminal_area.col(0).head(2) = Eigen::Vector2d(ct, st);
                terminal_area.col(0).tail(2) = Eigen::Vector2d(p.x, p.y);
                p.x = se2.x() + ct*h + st*w;
                p.y = se2.y() + st*h - ct*w;
                msg.points.push_back(p);
                msg.points.push_back(p);
                terminal_points.col(1) = Eigen::Vector2d(p.x, p.y) - terminal_points.col(0);
                terminal_area.col(1).head(2) = Eigen::Vector2d(st, -ct);
                terminal_area.col(1).tail(2) = Eigen::Vector2d(p.x, p.y);
                p.x = se2.x() - ct*h + st*w;
                p.y = se2.y() - st*h - ct*w;
                msg.points.push_back(p);
                msg.points.push_back(p);
                terminal_points.col(2) = Eigen::Vector2d(p.x, p.y) - terminal_points.col(0);
                terminal_area.col(2).head(2) = Eigen::Vector2d(-ct, -st);
                terminal_area.col(2).tail(2) = Eigen::Vector2d(p.x, p.y);
                p.x = se2.x() - ct*h - st*w;
                p.y = se2.y() - st*h + ct*w;
                msg.points.push_back(p);
                msg.points.push_back(p);
                terminal_points.col(3) = Eigen::Vector2d(p.x, p.y) - terminal_points.col(0);
                terminal_area.col(3).head(2) = Eigen::Vector2d(-st, ct);
                terminal_area.col(3).tail(2) = Eigen::Vector2d(p.x, p.y);
                p.x = se2.x() + ct*h - st*w;
                p.y = se2.y() + st*h + ct*w;
                msg.points.push_back(p);

                terminal_pub.publish(msg);
                return;
            }

            inline void setStateFromBox(const Eigen::Vector3d& box, Eigen::VectorXd& state)
            {
                state.resize(TRAILER_NUM+3);
                state.setConstant(box(2));
                Eigen::Vector2d half_box(box_length/2.0, box_width/2.0);
                state.head(2) = box.head(2) + Eigen::Vector2d(cos(box(2)), sin(box(2))) 
                                                * (box_length/2.0 - length[0] + rear_length - terminal_tol/2.0);
            }

            inline double stateError(const Eigen::VectorXd& state0, const Eigen::VectorXd& state1)
            {
                Eigen::VectorXd diff = (state0-state1).cwiseAbs();
                for (int i=0; i<1+TRAILER_NUM; i++)
                    diff(2+i) > M_PI ? diff(2+i) = PI_X_2 - diff(2+i) : diff(2+i);
                return diff.head(2).norm() + diff.tail(1+TRAILER_NUM).sum();
            }

            // utils: normalize angel to (-pi, pi]
            inline static void normYaw(double& yaw)
            {
                while (yaw>M_PI)
                    yaw -= PI_X_2;
                while (yaw<-M_PI)
                    yaw += PI_X_2;
                return;
            }

            inline static double dAngle(const double &angle1, const double &angle2)
            {
                double da = angle1 - angle2;
                normYaw(da);
                return da;
            }

            inline static void dis2Seg(const Eigen::Vector2d &start, 
                                    const Eigen::Vector2d &end,
                                    const Eigen::Vector2d &point, 
                                    Eigen::Vector2d & interact_p,
                                    double& dist)
            {
                Eigen::Vector2d v = end - start;
                Eigen::Vector2d w = point - start;
                double t = w.dot(v) / (v.squaredNorm());
                if (t < 0.0)
                    t = 0.0;
                else if (t > 1.0)
                    t = 1.0;

                interact_p = start + t * v;
                dist = (point - interact_p).norm();
                return;
            }

            // state: x0, y0, theta0, theta1, theta2, ...
            inline bool isJackKnife(const Eigen::VectorXd& state)
            {
                assert((size_t)state.size()==3+TRAILER_NUM);

                for (size_t i=0; i<TRAILER_NUM; i++)
                    if (fabs(dAngle(state(i+2), state(i+3))) > max_dtheta)
                        return true;
                
                return false;
            }

            // state: SE(2)0, SE(2)1, SE(2)2, ...
            inline bool isJackKnifeSE2(const Eigen::VectorXd& state)
            {
                assert((size_t)state.size()==3*TRAILER_NUM+3);

                for (size_t i=0; i<TRAILER_NUM; i++)
                    if (fabs(dAngle(state(i*3+2), state(i*3+5))) > max_dtheta)
                        return true;
                
                return false;
            }

            // state: x0, y0, theta0, theta1, theta2, ...
            // input: v, delta
            inline void stateTransitVel(const Eigen::VectorXd& state0,
                                        const Eigen::Vector2d& input,
                                        const double& T,
                                        Eigen::VectorXd& state1)
            {
                assert((size_t)state0.size()==3+TRAILER_NUM);

                double v = input(0);
                double w = v * tan(input(1)) / wheel_base;
                double y = T * w;

                state1.resize(state0.size());
                if (fabs(w) > 1e-4)
                {
                    state1(0) = state0(0) + v / w * (sin(state0(2)+y) - sin(state0(2)));
                    state1(1) = state0(1) - v / w * (cos(state0(2)+y) - cos(state0(2)));
                    state1(2) = state0(2) + y;
                    normYaw(state1(2));
                }
                else
                {
                    state1(0) = state0(0) + v * T * cos(state0(2));
                    state1(1) = state0(1) + v * T * sin(state0(2));
                    state1(2) = state0(2);
                }

                for (size_t i=0; i<TRAILER_NUM; i++)
                {
                    double sthetad = sin(state0(i+2)-state0(i+3));
                    double cthetad = cos(state0(i+2)-state0(i+3));
                    double w_temp = w;
                    w = (v * sthetad - Ltail[i] * w * cthetad) / Lhead[i];
                    v = v * cthetad + Ltail[i] * w_temp * sthetad;
                    state1(i+3) = state0(i+3) + w * T;
                    normYaw(state1(i+3));
                }

                return;
            }

            // x0, y0, theta0, theta1, ... ---> SE(2)0, SE(2)1, SE(2)2, ...
            inline void gainSE2State(const Eigen::VectorXd& state,
                                    Eigen::VectorXd& se2_state)
            {
                assert((size_t)state.size()==TRAILER_NUM+3);

                se2_state.resize(TRAILER_NUM*3+3);
                se2_state.head(3) = state.head(3);
                double ct = cos(state(2));
                double st = sin(state(2));
                Eigen::Vector2d traileri(state(0), state(1));
                for (size_t i=0; i<TRAILER_NUM; i++)
                {
                    Eigen::Vector2d joint(traileri(0)-Ltail[i]*ct, traileri(1)-Ltail[i]*st);
                    ct = cos(state(3+i));
                    st = sin(state(3+i));
                    traileri(0) = joint(0) - Lhead[i]*ct;
                    traileri(1) = joint(1) - Lhead[i]*st;
                    se2_state.segment(3*i+3, 2) = traileri;
                    se2_state(3*i+5) = state(i+3);
                }

                return;
            }

            // point: x, y
            inline bool isValid(const Eigen::VectorXd& state,
                                const Eigen::Vector2d& point)
            {
                if (isJackKnife(state))
                    return false;

                bool invalid = false;
                Eigen::Vector2d p_temp;

                double ct = cos(state(2));
                double st = sin(state(2));
                p_temp(0) = ct*point(0) - st*point(1) + state(0);
                p_temp(1) = st*point(0) + ct*point(1) + state(1);
                if (p_temp(0) > -rear_length && 
                    p_temp(0) < length[0] - rear_length &&
                    p_temp(1) > -0.5*width && p_temp(1) < 0.5*width)
                    invalid = true;
                else
                {
                    Eigen::Vector2d traileri(state(0), state(1));
                    for (size_t i=0; i<TRAILER_NUM; i++)
                    {
                        Eigen::Vector2d joint(traileri(0)-Ltail[i]*ct, traileri(1)-Ltail[i]*st);
                        ct = cos(state(3+i));
                        st = sin(state(3+i));
                        traileri(0) = joint(0) - Lhead[i]*ct;
                        traileri(1) = joint(1) - Lhead[i]*st;
                        p_temp(0) = ct*point(0) - st*point(1) + traileri(0);
                        p_temp(1) = st*point(0) + ct*point(1) + traileri(1);
                        if (p_temp(0) > -0.5 * length[i+1] && 
                            p_temp(0) < 0.5 * length[i+1] &&
                            p_temp(1) > -0.5*width && p_temp(1) < 0.5*width)
                        {
                            invalid = true;
                            break;
                        }
                    }
                }

                return !invalid;
            }

            // state: SE(2)0, SE(2)1, SE(2)2, ...
            // point: x, y
            inline bool isValidSE2(const Eigen::VectorXd& state,
                                    const Eigen::Vector2d& point)
            {
                assert((size_t)state.size()==3*TRAILER_NUM+3);

                for (size_t i=0; i<TRAILER_NUM; i++)
                {
                    if (fabs(dAngle(state(i*3+2), state(i*3+5))) > max_dtheta)
                        return false;
                }

                bool invalid = false;
                Eigen::Vector2d p_temp;

                double ct = cos(state(2));
                double st = sin(state(2));
                p_temp(0) = ct*point(0) - st*point(1) + state(0);
                p_temp(1) = st*point(0) + ct*point(1) + state(1);
                if (p_temp(0) > -rear_length && 
                    p_temp(0) < length[0] - rear_length &&
                    p_temp(1) > -0.5*width && p_temp(1) < 0.5*width)
                    invalid = true;
                else
                {
                    for (size_t i=0; i<TRAILER_NUM; i++)
                    {
                        ct = cos(state(3*i+5));
                        st = sin(state(3*i+5));
                        p_temp(0) = ct*point(0) - st*point(1) + state(3*i+3);
                        p_temp(1) = st*point(0) + ct*point(1) + state(3*i+4);
                        if (p_temp(0) > -0.5 * length[i+1] && 
                            p_temp(0) < 0.5 * length[i+1] &&
                            p_temp(1) > -0.5*width && p_temp(1) < 0.5*width)
                        {
                            invalid = true;
                            break;
                        }
                    }
                }

                return !invalid;
            }

            inline void getOnlySDF(const Eigen::VectorXd& state,
                                    const Eigen::Vector2d& point,
                                    double& sdf)
            {
                Eigen::VectorXd se2_state;
                gainSE2State(state, se2_state);
                sdf = 1e+19;

                for (size_t i = 0; i <TRAILER_NUM+1; i++)
                {
                    double yaw = se2_state(3*i+2);
                    Eigen::Matrix2d egoR;
                    egoR << cos(yaw), sin(yaw),
                            -sin(yaw), cos(yaw);
                    Eigen::Vector2d ego_point = egoR * (point - se2_state.segment(3*i, 2));
                    std::vector<Eigen::Vector2d> car_points;
                    for (int j=-1; j<=1; j+=2)
                        for (int k=-1; k<=1; k+=2)
                        {
                            Eigen::Vector2d step(j*length[i] / 2.0, k*width / 2.0);
                            if (i == 0)
                            {
                                if (j > 0)
                                    step(0) = length[i] - rear_length;
                                else
                                    step(0) = -rear_length;
                            }
                                
                            car_points.push_back(step);
                        }
                    double dist = 1e+19;
                    double d_temp;
                    Eigen::Vector2d interact_p;
                    dis2Seg(car_points[0], car_points[1], ego_point, interact_p, d_temp);
                    if (d_temp < dist)
                        dist = d_temp;
                    dis2Seg(car_points[0], car_points[2], ego_point, interact_p, d_temp);
                    if (d_temp < dist)
                        dist = d_temp;
                    dis2Seg(car_points[3], car_points[2], ego_point, interact_p, d_temp);
                    if (d_temp < dist)
                        dist = d_temp;
                    dis2Seg(car_points[3], car_points[1], ego_point, interact_p, d_temp);
                    if (d_temp < dist)
                        dist = d_temp;
                    if (ego_point[0] > car_points[0][0] &&
                        ego_point[0] < car_points[2][0] &&
                        ego_point[1] > car_points[0][1] &&
                        ego_point[1] < car_points[1][1])
                    {
                        sdf = -dist;
                        break;
                    }
                    if (sdf > dist)
                        sdf = dist;
                    if (i < TRAILER_NUM)
                    {
                        dis2Seg(se2_state.segment(3*i, 2), se2_state.segment(3*i+3, 2), point, interact_p, dist);
                        if (sdf > dist)
                            sdf = dist;
                    }
                }
                return;
            }
                    
            inline void getOnlyGrad(const Eigen::VectorXd& state,
                                    const Eigen::Vector2d& point,
                                    Eigen::Vector2d& grad)
            {
                Eigen::VectorXd se2_state;
                gainSE2State(state, se2_state);
                double sdf = 1e+19;

                for (size_t i = 0; i <TRAILER_NUM+1; i++)
                {
                    double yaw = se2_state(3*i+2);
                    Eigen::Matrix2d egoR;
                    egoR << cos(yaw), sin(yaw),
                            -sin(yaw), cos(yaw);
                    Eigen::Vector2d ego_point = egoR * (point - se2_state.segment(3*i, 2));
                    std::vector<Eigen::Vector2d> car_points;
                    for (int j=-1; j<=1; j+=2)
                        for (int k=-1; k<=1; k+=2)
                        {
                            Eigen::Vector2d step(j*length[i] / 2.0, k*width / 2.0);
                            if (i == 0)
                            {
                                if (j > 0)
                                    step(0) = length[i] - rear_length;
                                else
                                    step(0) = -rear_length;
                            }
                            car_points.push_back(step);
                        }
                    double dist = 1e+19;
                    Eigen::Vector2d c_temp, v_temp;
                    double d_temp;
                    Eigen::Vector2d interact_p;
                    dis2Seg(car_points[0], car_points[1], ego_point, interact_p, d_temp);
                    if (d_temp < dist)
                    {
                        dist = d_temp;
                        c_temp = interact_p;
                        v_temp = car_points[0] - car_points[1];
                    }
                    dis2Seg(car_points[0], car_points[2], ego_point, interact_p, d_temp);
                    if (d_temp < dist)
                    {
                        dist = d_temp;
                        c_temp = interact_p;
                        v_temp = car_points[2] - car_points[0];
                    }
                    dis2Seg(car_points[3], car_points[2], ego_point, interact_p, d_temp);
                    if (d_temp < dist)
                    {
                        dist = d_temp;
                        c_temp = interact_p;
                        v_temp = car_points[3] - car_points[2];
                    }
                    dis2Seg(car_points[3], car_points[1], ego_point, interact_p, d_temp);
                    if (d_temp < dist)
                    {
                        dist = d_temp;
                        c_temp = interact_p;
                        v_temp = car_points[1] - car_points[3];
                    }
                    if (ego_point[0] > car_points[0][0] &&
                        ego_point[0] < car_points[2][0] &&
                        ego_point[1] > car_points[0][1] &&
                        ego_point[1] < car_points[1][1])
                    {
                        Eigen::Vector2d grad_vec = egoR.transpose() * c_temp + se2_state.segment(3*i, 2) - point;
                        if (grad_vec.norm() < 1e-4)
                            grad = egoR.transpose() * Eigen::Vector2d(v_temp(1), -v_temp(0)).normalized();
                        else
                            grad = grad_vec.normalized();
                        break;
                    }
                    if (sdf > dist)
                    {
                        sdf = dist;
                        Eigen::Vector2d grad_vec = point - egoR.transpose() * c_temp - se2_state.segment(3*i, 2);
                        if (grad_vec.norm() < 1e-4)
                            grad = egoR.transpose() * Eigen::Vector2d(v_temp(1), -v_temp(0)).normalized();
                        else
                            grad = grad_vec.normalized();
                    }
                    if (i < TRAILER_NUM)
                    {
                        dis2Seg(se2_state.segment(3*i, 2), se2_state.segment(3*i+3, 2), point, interact_p, dist);
                        if (sdf > dist)
                        {
                            sdf = dist;
                            Eigen::Vector2d grad_vec = point - interact_p;
                            grad = grad_vec.normalized();
                        }
                    }
                }
                return;
            }
            
            inline void getSDFWithGrad(const Eigen::VectorXd& state,
                                        const Eigen::Vector2d& point,
                                        double& sdf,
                                        Eigen::Vector2d& grad)
            {
                Eigen::VectorXd se2_state;
                gainSE2State(state, se2_state);
                sdf = 1e+19;

                for (size_t i = 0; i <TRAILER_NUM+1; i++)
                {
                    double yaw = se2_state(3*i+2);
                    Eigen::Matrix2d egoR;
                    egoR << cos(yaw), sin(yaw),
                            -sin(yaw), cos(yaw);
                    Eigen::Vector2d ego_point = egoR * (point - se2_state.segment(3*i, 2));
                    std::vector<Eigen::Vector2d> car_points;
                    for (int j=-1; j<=1; j+=2)
                        for (int k=-1; k<=1; k+=2)
                        {
                            Eigen::Vector2d step(j*length[i] / 2.0, k*width / 2.0);
                            if (i == 0)
                            {
                                if (j > 0)
                                    step(0) = length[i] - rear_length;
                                else
                                    step(0) = -rear_length;
                            }
                            car_points.push_back(step);
                        }
                    double dist = 1e+19;
                    Eigen::Vector2d c_temp, v_temp;
                    double d_temp;
                    Eigen::Vector2d interact_p;
                    dis2Seg(car_points[0], car_points[1], ego_point, interact_p, d_temp);
                    if (d_temp < dist)
                    {
                        dist = d_temp;
                        c_temp = interact_p;
                        v_temp = car_points[0] - car_points[1];
                    }
                    dis2Seg(car_points[0], car_points[2], ego_point, interact_p, d_temp);
                    if (d_temp < dist)
                    {
                        dist = d_temp;
                        c_temp = interact_p;
                        v_temp = car_points[2] - car_points[0];
                    }
                    dis2Seg(car_points[3], car_points[2], ego_point, interact_p, d_temp);
                    if (d_temp < dist)
                    {
                        dist = d_temp;
                        c_temp = interact_p;
                        v_temp = car_points[3] - car_points[2];
                    }
                    dis2Seg(car_points[3], car_points[1], ego_point, interact_p, d_temp);
                    if (d_temp < dist)
                    {
                        dist = d_temp;
                        c_temp = interact_p;
                        v_temp = car_points[1] - car_points[3];
                    }
                    if (ego_point[0] > car_points[0][0] &&
                        ego_point[0] < car_points[2][0] &&
                        ego_point[1] > car_points[0][1] &&
                        ego_point[1] < car_points[1][1])
                    {
                        sdf = -dist;
                        Eigen::Vector2d grad_vec = egoR.transpose() * c_temp + se2_state.segment(3*i, 2) - point;
                        if (grad_vec.norm() < 1e-4)
                            grad = egoR.transpose() * Eigen::Vector2d(v_temp(1), -v_temp(0)).normalized();
                        else
                            grad = grad_vec.normalized();
                        break;
                    }
                    if (sdf > dist)
                    {
                        sdf = dist;
                        Eigen::Vector2d grad_vec = point - egoR.transpose() * c_temp - se2_state.segment(3*i, 2);
                        if (grad_vec.norm() < 1e-4)
                            grad = egoR.transpose() * Eigen::Vector2d(v_temp(1), -v_temp(0)).normalized();
                        else
                            grad = grad_vec.normalized();
                    }
                    if (i < TRAILER_NUM)
                    {
                        dis2Seg(se2_state.segment(3*i, 2), se2_state.segment(3*i+3, 2), point, interact_p, dist);
                        if (sdf > dist)
                        {
                            sdf = dist;
                            grad = (point - interact_p).normalized();
                        }
                    }
                }
                return;
            }

            typedef shared_ptr<Trailer> Ptr;
            typedef unique_ptr<Trailer> UniPtr;
    };
}
