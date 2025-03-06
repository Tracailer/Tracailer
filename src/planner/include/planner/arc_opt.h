#pragma once

#include <thread>
#include <numeric>
#include <unordered_map>

#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

#include "utils/minco.hpp"
#include "utils/lbfgs.hpp"
#include "planner/trailer.hpp"
#include "planner/grid_map.h"

#define ESDF 0

namespace trailer_planner
{
    class ArcTraj
    {
        public:
            PolyTrajectory<1, 5> arc;
            PolyTrajectory<2, 5> head;
            PolyTrajectory<TRAILER_NUM, 5> tails;

        public:
            double getTotalDuration() const
            {
                return std::min(arc.getTotalDuration(), tails.getTotalDuration());
            }

            Eigen::VectorXd getState(double t) const
            {
                t = std::max(0.0, std::min(t, getTotalDuration()));
                Eigen::VectorXd state;
                state.resize(3+TRAILER_NUM);
                double a = arc.getPos(t)[0];
                state.head(2) = head.getPos(a);
                Eigen::Vector2d vel = head.getVel(a);
                state(2) = atan2(vel(1), vel(0));
                state.tail(TRAILER_NUM) = tails.getPos(t);
                return state;
            }
    };

    class ArcOpt
    {
        public:
            // params

            /// problem
            int    collision_type;  // 0: ESDF;
            double corridor_limit;
            double piece_len;
            double rho_T;
            double piece_times;
            double max_vel;
            double max_alon;
            double max_alat;
            double max_angular_vel;
            double max_kappa;
            double max_thetad;
            double safe_threshold;
            double inner_weight_jerk;
            double inner_weight_kinetics;
            double inner_weight_vel;
            double inner_weight_min_vel;
            double inner_weight_alon;
            double inner_weight_alat;
            double inner_weight_kappa;
            double inner_weight_collision;
            double inner_weight_tail;

            /// L-BFGS
            double g_epsilon;
            double min_step;
            double inner_max_iter;
            double delta;
            int    mem_size;
            int    past;
            int    int_K;

            /// ALM
            bool   use_scaling;
            double rho;
            double rho_init;
            double beta;
            double gamma;
            double epsilon_con;
            double max_iter;

            /// debug and test
            bool in_debug;
            double planning_time = 0.0;
            double traj_len = 0.0;
            double traj_time = 0.0;
            double mean_a = 0.0;
            double mean_j = 0.0;
            double mean_kappa = 0.0;

            // data
            int             piece_num_pos;
            int             piece_num_theta;
            int             dim_time;
            double          equal_num;
            double          non_equal_num;
            double          scale_fx;
            double          start_v;
            Eigen::VectorXd lambda;
            Eigen::VectorXd mu;
            Eigen::VectorXd hx;
            Eigen::VectorXd gx;
            Eigen::VectorXd scale_cx;
            Eigen::MatrixXd init_pos;
            Eigen::MatrixXd end_pos;
            Eigen::MatrixXd init_theta;
            Eigen::MatrixXd end_theta;
            Trailer::Ptr    trailer;
            GridMap::Ptr    grid_map;
            MinJerkOpt<1>   arc_opt;
            MinJerkOpt<2>   head_opt;
            MinJerkOpt<TRAILER_NUM>  tails_opt;
            std::vector<Eigen::MatrixXd> corridors;
            int thread_idx = 0;

            // ros
            ros::Publisher debug_pub;
            ros::Publisher corridor_pub;

            void init(ros::NodeHandle& nh);

            bool optimizeTraj(std::vector<Eigen::VectorXd> init_path, 
                              double init_vel = 0.0, int piece_num = 0, 
                              Eigen::MatrixXd local_pts = Eigen::MatrixXd::Zero(3, 3));

            static double innerCallback(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad);

            static int earlyExit(void* ptrObj, const Eigen::VectorXd& x, const Eigen::VectorXd& grad, 
                                const double fx, const double step, int k, int ls);

            void calTailStateCostGrad(double& cost, Eigen::VectorXd& gdTail, const Eigen::VectorXd& dtheta_end,
                                    const Eigen::Vector3d& Tailtractor_temp, Eigen::Vector3d& gradTailtractor_temp);
            
            void calConstrainCostGrad(double& cost, Eigen::MatrixXd& gdCpos, Eigen::VectorXd &gdApos,
                                                    Eigen::MatrixXd& gdCtheta, Eigen::VectorXd &gdTtheta,
                                                    Eigen::MatrixXd& gdCarc, Eigen::VectorXd &gdTarc);

            void pubDebugTraj(const ArcTraj& traj) const
            {
                double scale = 0.03;
                visualization_msgs::MarkerArray debug_msg;

                double dur = traj.getTotalDuration();
                std::vector<Eigen::VectorXd> se2_path;
                std::vector<Eigen::VectorXd> se2_path_node;
                for (double i = 0; i <= dur - 1e-4; i+=0.01)
                {
                    Eigen::VectorXd se2_state;
                    Eigen::VectorXd state = traj.getState(i);
                    trailer->gainSE2State(state, se2_state);
                    se2_path.push_back(se2_state);
                }
                Eigen::VectorXd ts = traj.arc.getDurations();
                double time = 0.0;
                for (int i=0; i<ts.size(); i++)
                {
                    time += ts(i);
                    Eigen::VectorXd se2_state;
                    Eigen::VectorXd state = traj.getState(time);
                    trailer->gainSE2State(state, se2_state);
                    se2_path_node.push_back(se2_state);
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
                    sphere.color.r = line_strip.color.r = trailer->array_msg.markers[i].color.r;
                    sphere.color.g = line_strip.color.g = trailer->array_msg.markers[i].color.g;
                    sphere.color.b = line_strip.color.b = trailer->array_msg.markers[i].color.b;
                    sphere.color.a = line_strip.color.a = 1;
                    sphere.scale.x = scale * 2;
                    sphere.scale.y = scale * 2;
                    sphere.scale.z = scale * 2;
                    line_strip.scale.x = scale / 2;
                    geometry_msgs::Point pt;
                    
                    for (auto p:se2_path)
                    {
                        pt.x = p[3*i];
                        pt.y = p[3*i+1];
                        pt.z = 0.0;
                        line_strip.points.push_back(pt);
                    }
                    for (auto p:se2_path_node)
                    {
                        pt.x = p[3*i];
                        pt.y = p[3*i+1];
                        pt.z = 0.0;
                        sphere.points.push_back(pt);
                    }
                    debug_msg.markers.push_back(line_strip);
                    debug_msg.markers.push_back(sphere);
                }
                debug_pub.publish(debug_msg);
            }

        public:
            inline void setTrailerEnv(Trailer::Ptr p, GridMap::Ptr g)
            {
                trailer = p;
                grid_map = g;
                return;
            }

            inline ArcTraj getTraj() const
            {
                ArcTraj arc_traj;
                arc_traj.arc = arc_opt.getTraj();
                arc_traj.head = head_opt.getTraj();
                arc_traj.tails = tails_opt.getTraj();
                return arc_traj;
            }

            inline void printConstraintsSituations(const ArcTraj& traj)
            {
                double min_vel_ = 1000.0;
                double min_pvel_ = 1000.0;
                double min_darc_ = 1000.0;
                double max_vel_ = 0.0;
                double max_alon_ = 0.0;
                double max_alat_ = 0.0;
                double max_kappa_ = 0.0;
                double kinetics_error = 0.0;
                Eigen::VectorXd max_w_, max_thetad_;
                max_w_.resize(TRAILER_NUM);
                max_thetad_.resize(TRAILER_NUM);
                max_w_.setZero();
                max_thetad_.setZero();

                traj_len = 0.0;
                traj_time = traj.getTotalDuration();
                mean_a = 0.0;
                mean_j = 0.0;
                mean_kappa = 0.0;
                double num_wps = 0.0;

                double dt = 0.07;
                // double dt = 0.01;
                Eigen::Vector2d temp_pos;
                for(double t = 0.0; t < traj.getTotalDuration(); t += dt)
                {
                    double arc = traj.arc.getPos(t)[0];
                    double darc = traj.arc.getVel(t)[0];
                    double d2arc = traj.arc.getAcc(t)[0];
                    double d3arc = traj.arc.getJer(t)[0];
                    Eigen::Vector2d pos = traj.head.getPos(arc);
                    Eigen::Vector2d vel = traj.head.getVel(arc);
                    Eigen::Vector2d acc = traj.head.getAcc(arc);
                    Eigen::Vector2d jer = traj.head.getJer(arc);

                    Eigen::Vector2d pos_head;
                    double vlon = vel.norm();
                    double inv_vlon = 1.0 / vlon;
                    double inv_vlon_2 = inv_vlon * inv_vlon;

                    double alon = vlon*d2arc + vel.dot(acc) * inv_vlon * darc * darc;
                    double alat = (acc(1)*vel(0) - acc(0)*vel(1)) * inv_vlon * darc * darc;
                    double kappa = (acc(1)*vel(0) - acc(0)*vel(1)) * inv_vlon * inv_vlon_2;
                    double theta0 = atan2(vel(1), vel(0));
                    Eigen::VectorXd tail_thetas = traj.tails.getPos(t);
                    Eigen::VectorXd tail_thetas_l;
                    if (TRAILER_NUM > 1)
                        tail_thetas_l = tail_thetas.head(TRAILER_NUM-1) - tail_thetas.tail(TRAILER_NUM-1);
                    Eigen::VectorXd ws = traj.tails.getVel(t);

                    double real_vlon = vlon * darc;
                    kinetics_error += fabs(real_vlon*sin(theta0-tail_thetas(0)) - trailer->Lhead[0]*ws[0]) * dt;
                    double vel_now = real_vlon;
                    for (int i=1; i<TRAILER_NUM; i++)
                    {
                        vel_now = vel_now * cos(ws[i]-ws[i-1]);
                        kinetics_error += fabs(vel_now*sin(tail_thetas[i-1]-tail_thetas[i]) - trailer->Lhead[i]*ws[i]) * dt;
                    }

                    Eigen::Vector2d cacc(acc(0)*darc+vel(0)*d2arc, acc(1)*darc+vel(1)*d2arc);
                    Eigen::Vector2d cjerk(jer(0)*darc+2.0*acc(0)*d2arc+vel(0)*d3arc, 
                                         jer(1)*darc+2.0*acc(1)*d2arc+vel(1)*d3arc);
                    mean_a += cacc.norm() * dt;
                    mean_j += cjerk.norm() * dt;
                    mean_kappa += fabs(kappa) * dt;
                    num_wps += 1.0;
                    if (t == 0.0)
                        temp_pos = pos;
                    else
                    {
                        traj_len += (pos - temp_pos).norm();
                        temp_pos = pos;
                    }

                    if(fabs(min_vel_) > fabs(real_vlon))
                        min_vel_ = real_vlon;
                    if(fabs(min_pvel_) > fabs(vlon))
                        min_pvel_ = vlon;
                    if(fabs(max_vel_) < fabs(real_vlon))
                        max_vel_ = real_vlon;
                    if(fabs(max_alon_) < fabs(alon))
                        max_alon_ = alon;
                    if(fabs(max_alat_) < fabs(alat))
                        max_alat_ = alat;
                    if (fabs(max_kappa_) < fabs(kappa))
                        max_kappa_ = kappa;
                    if (min_darc_ > darc)
                        min_darc_ = darc;

                    for (size_t i=0; i<TRAILER_NUM; i++)
                        if (fabs(max_w_[i]) < fabs(ws[i]))
                            max_w_[i] = ws[i];
                    
                    double thetad_ = theta0 - tail_thetas[0];
                    trailer->normYaw(thetad_);
                    if (fabs(max_thetad_[0]) < fabs(thetad_))
                        max_thetad_[0] = thetad_;
                    for (size_t i=0; i<TRAILER_NUM-1; i++)
                    {
                        trailer->normYaw(tail_thetas_l[i]);
                        if (fabs(max_thetad_[i+1]) < fabs(tail_thetas_l[i]))
                            max_thetad_[i+1] = tail_thetas_l[i];
                    }
                }

                mean_a /= traj_time;
                mean_j /= traj_time;
                mean_kappa /= traj_time;

                PRINT_GREEN("[Arc Optimizer] Print Constraints Situations:");
                PRINT_GREEN("            0. Entirety:");
                PRINTF_WHITE("            max theta diff   : ");
                if (max_thetad_.maxCoeff() > 1.01 * max_thetad)
                    PRINT_RED(std::to_string(max_thetad_.maxCoeff()));
                else
                    PRINTF_WHITE(std::to_string(max_thetad_.maxCoeff())+"\n");
                PRINTF_WHITE("            kinetics error   : ");
                PRINTF_WHITE(std::to_string(kinetics_error)+"\n");

                PRINT_GREEN("            1. Tractor:");
                PRINTF_WHITE("            max velocity     : ");
                if (fabs(max_vel_) > 1.01 * max_vel)
                    PRINT_RED(std::to_string(max_vel_));
                else
                    PRINTF_WHITE(std::to_string(max_vel_)+"\n");

                PRINTF_WHITE("            min darc         : ");
                if (min_darc_ < 0.0)
                    PRINT_RED(std::to_string(min_darc_));
                else
                    PRINTF_WHITE(std::to_string(min_darc_)+"\n");
                
                PRINTF_WHITE("            min pseudo vel   : ");
                if (min_pvel_ < 0.0)
                    PRINT_RED(std::to_string(min_pvel_));
                else
                    PRINTF_WHITE(std::to_string(min_pvel_)+"\n");

                PRINTF_WHITE("            min velocity     : ");
                if (min_vel_ < 0.0)
                    PRINT_RED(std::to_string(min_vel_));
                else
                    PRINTF_WHITE(std::to_string(min_vel_)+"\n");

                PRINTF_WHITE("            max acc longitude: ");
                if (fabs(max_alon_) > 1.01 * max_alon)
                    PRINT_RED(std::to_string(max_alon_));
                else
                    PRINTF_WHITE(std::to_string(max_alon_)+"\n");

                PRINTF_WHITE("            max acc latitude : ");
                if (fabs(max_alat_) > 1.01 * max_alat)
                    PRINT_RED(std::to_string(max_alat_));
                else
                    PRINTF_WHITE(std::to_string(max_alat_)+"\n");

                PRINTF_WHITE("            max curvature    : ");
                if (fabs(max_kappa_) > 1.01 * max_kappa)
                    PRINT_RED(std::to_string(max_kappa_));
                else
                    PRINTF_WHITE(std::to_string(max_kappa_)+"\n");

                PRINT_GREEN("            2. Trailers:");
                PRINTF_WHITE("            max angular vel  : ");
                if (max_w_.maxCoeff() > 1.01 * max_angular_vel)
                    PRINT_RED(std::to_string(max_w_.maxCoeff()));
                else
                    PRINTF_WHITE(std::to_string(max_w_.maxCoeff())+"\n");
                PRINT_GREEN("            3. Aggresiveness:");
                PRINTF_WHITE("            traj duration    : "+std::to_string(traj.getTotalDuration())+"s\n");
                
                return;
            }

            // update dual varables
            inline void updateDualVars()
            {
                lambda += rho * hx;
                for(int i = 0; i < non_equal_num; i++)
                    mu(i) = std::max(mu(i)+rho*gx(i), 0.0);
                rho = std::min((1 + gamma) * rho, beta);
            }

            // convergence judgement
            inline bool judgeConvergence()
            {
                // PRINTF_WHITE("[ALM] hx_inf_norm: "+to_string(hx.lpNorm<Eigen::Infinity>())+" gx_inf_norm: "+to_string(gx.cwiseMax(-mu/rho).lpNorm<Eigen::Infinity>())+"\n");
                // PRINTF_WHITE("[ALM] lambda: "+to_string(lambda.lpNorm<Eigen::Infinity>())+" mu: "+to_string(mu.lpNorm<Eigen::Infinity>())+"\n\n");
                
                if (std::max(hx.lpNorm<Eigen::Infinity>(), \
                            gx.cwiseMax(-mu/rho).lpNorm<Eigen::Infinity>()) < epsilon_con)
                {
                    return true;
                }

                return false;
            }

            // get 0.5ρ(h + λ/ρ)^2 or 0.5ρ(g + μ/ρ)^2
            inline double getAugmentedCost(double h_or_g, double lambda_or_mu)
            {
                return h_or_g * (lambda_or_mu + 0.5*rho*h_or_g);
            }
            
            // get ρh+λ or ρg+μ, the gradient of `getAugmentedCost`
            inline double getAugmentedGrad(double h_or_g, double lambda_or_mu)
            {
                return rho * h_or_g + lambda_or_mu;
            }
            
            // θ = sigmoid(φ)
            inline double sigmoidC2(const double& vphi)
            {
                double e_ang = expC2(vphi);
                return 2.0 * max_thetad * e_ang / (1.0 + e_ang) - max_thetad;
            }

            // θ = sigmoid(φ)
            inline static double sigmoidC2(const double& vphi, double max_thetad)
            {
                double e_ang = expC2(vphi);
                return 2.0 * max_thetad * e_ang / (1.0 + e_ang) - max_thetad;
            }

            // φ = invSigmoid(θ)
            inline double invSigmoidC2(const double& theta)
            {
                double b = 0.5 * (max_thetad + theta) / max_thetad;
                return logC2(b/(1-b));
            }

            // get dθ/dφ
            inline double getThetatoVphiGrad(const double& vphi)
            {
                double e_ang_1 = expC2(vphi) + 1.0;
                return 2.0 * max_thetad * getTtoTauGrad(vphi) / (e_ang_1 * e_ang_1);
            }

            // T = e^τ
            inline static double expC2(const double& tau)
            {
                return tau > 0.0 ? ((0.5 * tau + 1.0) * tau + 1.0) : 1.0 / ((0.5 * tau - 1.0) * tau + 1.0);
            }

            // τ = ln(T)
            inline static double logC2(const double& T)
            {
                return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
            }

            // get dT/dτ
            inline double getTtoTauGrad(const double& tau)
            {
                if (tau > 0)
                    return tau + 1.0;
                else 
                {
                    double denSqrt = (0.5 * tau - 1.0) * tau + 1.0;
                    return (1.0 - tau) / (denSqrt * denSqrt);
                } 
            }

            // know τ
            // then get T (uniform)
            inline static void calTfromTauUni(const double& tau, Eigen::VectorXd& T)
            {
                T.setConstant(expC2(tau) / T.size());
                return;
            }

            // know τ
            // then get T
            inline static void calTfromTau(const Eigen::VectorXd& tau, Eigen::VectorXd& T)
            {
                T.resize(tau.size());
                for (int i=0; i<tau.size(); i++)
                {
                    T(i) = expC2(tau(i));
                }
                return;
            }

            // know dJ/dP
            // then get dJ/dξ = dJ/dP * dP/dξ
            inline void calGradPtoGradXi(const Eigen::Matrix2Xd& gradP, \
                                        const Eigen::VectorXd& xi, \
                                        Eigen::Matrix4Xd& gradXi)
            {
                gradXi.resize(4, TRAILER_NUM+1);
                for (int i=0; i<TRAILER_NUM+1; i++)
                {
                    Eigen::Vector4d xii = xi.segment(4*i, 4);
                    double normInv = 1.0 / xii.norm();
                    Eigen::VectorXd unitQ = xii * normInv;
                    Eigen::VectorXd gradQ;
                    gradQ.resize(4);
                    gradQ.head(3) = (trailer->terminal_points.rightCols(3).transpose() * gradP.col(i)).array() *
                                    unitQ.head(3).array() * 2.0;
                    gradQ(3) = 0.0;
                    gradXi.col(i) = (gradQ - unitQ * unitQ.dot(gradQ)) * normInv;
                }
                return;
            }

            // know ξ
            // then get P
            inline void calPfromXi(const Eigen::VectorXd& xi, Eigen::Matrix2Xd& p)
            {
                p.resize(2, TRAILER_NUM+1);
                for (int i=0; i<TRAILER_NUM+1; i++)
                {
                    Eigen::Vector4d xii = xi.segment(4*i, 4);
                    Eigen::Vector3d xii1 = xii.normalized().head(3);
                    p.col(i) = trailer->terminal_points.rightCols(3) * xii1.cwiseProduct(xii1) + trailer->terminal_points.col(0);
                }
                return;
            }

            // reLu
            inline void smoothL1Penalty(const double& x, double& f, double &df)
            {
                const double miu = 1.0e-4;
                
                if (x > miu)
                {
                    df = 1.0; 
                    f =  x - 0.5 * miu;
                }
                else
                {
                    double xdmu = x / miu;
                    double sqrxdmu = xdmu * xdmu;
                    double mumxd2 = miu - 0.5 * x;
                    f = mumxd2 * sqrxdmu * xdmu;
                    df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / miu);
                }

                return;
            }

            inline double getDurationTrapezoid(const double &length, 
                                               const double &startV, const double &endV, 
                                               const double &maxV, const double &maxA)
            {
                double critical_len; 
                double startv2 = startV * startV;
                double endv2 = endV * endV;
                double maxv2 = maxV * maxV;
                if(startV>maxV)
                    startv2 = maxv2;
                if(endV>maxV)
                    endv2 = maxv2;
                critical_len = (maxv2-startv2)/(2*maxA)+(maxv2-endv2)/(2*maxA);
                if(length>=critical_len)
                    return (maxV-startV)/maxA+(maxV-endV)/maxA+(length-critical_len)/maxV;
                else
                {
                    double tmpv = sqrt(0.5*(startv2+endv2+2*maxA*length));
                    return (tmpv-startV)/maxA + (tmpv-endV)/maxA;
                }
                return 0.0;
            }

            inline double getArcTrapezoid(const double &curt, const double &locallength, 
                                          const double &startV, const double &endV, 
                                          const double &maxV, const double &maxA)
            {
                double critical_len; 
                double startv2 = startV * startV;
                double endv2 = endV * endV;
                double maxv2 = maxV * maxV;
                if(startV>maxV)
                    startv2 = maxv2;
                if(endV>maxV)
                    endv2 = maxv2;
                critical_len = (maxv2-startv2)/(2*maxA)+(maxv2-endv2)/(2*maxA);
                if(locallength>=critical_len)
                {
                    double t1 = (maxV-startV)/maxA;
                    double t2 = t1+(locallength-critical_len)/maxV;
                    if(curt<=t1)
                        return startV*curt + 0.5*maxA*(curt*curt);
                    else if(curt<=t2)
                        return startV*t1 + 0.5*maxA*(t1*t1)+(curt-t1)*maxV;
                    else
                        return startV*t1 + 0.5*maxA*(t1*t1) + (t2-t1)*maxV + maxV*(curt-t2)-0.5*maxA*(curt-t2)*(curt-t2);
                }
                else
                {
                    double tmpv = sqrt(0.5*(startv2+endv2+2*maxA*locallength));
                    double tmpt = (tmpv-startV)/maxA;
                    if(curt<=tmpt)
                        return startV*curt+0.5*maxA*(curt*curt);
                    else
                        return startV*tmpt+0.5*maxA*(tmpt*tmpt) + tmpv*(curt-tmpt)-0.5*maxA*(curt-tmpt)*(curt-tmpt);
                }
                return 0.0;
            }
    };
}
