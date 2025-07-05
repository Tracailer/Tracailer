#include "planner/arc_opt.h"

namespace trailer_planner
{
    void ArcOpt::init(ros::NodeHandle& nh)
    {
        nh.getParam("arc_opt/rho_T", rho_T);
        nh.getParam("arc_opt/piece_times", piece_times);
        nh.getParam("arc_opt/collision_type", collision_type);
        nh.getParam("arc_opt/corridor_limit", corridor_limit);
        nh.getParam("arc_opt/piece_len", piece_len);
        nh.getParam("arc_opt/max_vel", max_vel);
        nh.getParam("arc_opt/max_alon", max_alon);
        nh.getParam("arc_opt/max_alat", max_alat);
        nh.getParam("arc_opt/max_angular_vel", max_angular_vel);
        nh.getParam("arc_opt/max_kappa", max_kappa);
        nh.getParam("arc_opt/max_thetad", max_thetad);
        nh.getParam("arc_opt/safe_threshold", safe_threshold);
        nh.getParam("arc_opt/g_epsilon", g_epsilon);
        nh.getParam("arc_opt/min_step", min_step);
        nh.getParam("arc_opt/inner_max_iter", inner_max_iter);
        nh.getParam("arc_opt/delta", delta);
        nh.getParam("arc_opt/mem_size", mem_size);
        nh.getParam("arc_opt/past", past);
        nh.getParam("arc_opt/int_K", int_K);
        nh.getParam("arc_opt/in_debug", in_debug);
        nh.getParam("arc_opt/rho", rho_init);
        nh.getParam("arc_opt/beta", beta);
        nh.getParam("arc_opt/gamma", gamma);
        nh.getParam("arc_opt/epsilon_con", epsilon_con);
        nh.getParam("arc_opt/max_iter", max_iter);
        nh.getParam("arc_opt/use_scaling", use_scaling);
        nh.getParam("arc_opt/inner_weight_jerk", inner_weight_jerk);
        nh.getParam("arc_opt/inner_weight_kinetics", inner_weight_kinetics);
        nh.getParam("arc_opt/inner_weight_vel", inner_weight_vel);
        nh.getParam("arc_opt/inner_weight_min_vel", inner_weight_min_vel);
        nh.getParam("arc_opt/inner_weight_alon", inner_weight_alon);
        nh.getParam("arc_opt/inner_weight_alat", inner_weight_alat);
        nh.getParam("arc_opt/inner_weight_kappa", inner_weight_kappa);
        nh.getParam("arc_opt/inner_weight_collision", inner_weight_collision);
        nh.getParam("arc_opt/inner_weight_tail", inner_weight_tail);

        debug_pub = nh.advertise<visualization_msgs::MarkerArray>("/flat_poly/debug_path", 1);

        return;
    }

    bool ArcOpt::optimizeTraj(std::vector<Eigen::VectorXd> init_path, 
                              Eigen::Vector2d init_vw, int piece_num,
                              Eigen::MatrixXd local_pts)
    {
        if (init_path.empty())
            return false;

        // smooth yaw
        for (size_t j=0; j<TRAILER_NUM+1; j++)
        {
            double dyaw1;
            for (size_t i=0; i<init_path.size()-1; i++)
            {
                dyaw1 = init_path[i+1][2+j] - init_path[i][2+j];
                while (dyaw1 >= M_PI / 2)
                {
                    init_path[i+1][2+j] -= M_PI * 2;
                    dyaw1 = init_path[i+1][2+j] - init_path[i][2+j];
                }
                while (dyaw1 <= -M_PI / 2)
                {
                    init_path[i+1][2+j] += M_PI * 2;
                    dyaw1 = init_path[i+1][2+j] - init_path[i][2+j];
                }
            }
        }

        // PRINT_GREEN("opt init path: ");
        // for (size_t i=0; i<init_path.size(); i++)
        // {
        //     PRINT_GREEN(init_path[i].transpose());
        // }

        // boundary condition
        init_pos = Eigen::MatrixXd::Zero(2, 3);
        end_pos = Eigen::MatrixXd::Zero(2, 3);
        init_theta = Eigen::MatrixXd::Zero(TRAILER_NUM, 3);
        end_theta = Eigen::MatrixXd::Zero(TRAILER_NUM, 3);
        Eigen::VectorXd end_se2_state = init_path.back();
        init_pos.col(0) = init_path[0].head(2);
        end_pos.col(0) = init_path.back().head(2);
        init_pos.col(1) << cos(init_path[0][2]), sin(init_path[0][2]);
        end_pos.col(1) << cos(init_path.back()[2]), sin(init_path.back()[2]);
        init_theta.col(0) = init_path[0].segment(3, TRAILER_NUM);
        end_theta.col(0) = init_path.back().segment(3, TRAILER_NUM);
        start_v = init_vw(0);
        double temp_vel = init_vw(0);
        double temp_w = init_vw(1);
        for (size_t i=0; i<TRAILER_NUM; i++)
        {
            init_theta.col(1)[i] = (temp_vel * sin(init_path[0][2+i] - init_path[0][3+i]) 
                                    - temp_w * cos(init_path[0][2+i] - init_path[0][3+i]) * trailer->Ltail[i])
                                    / trailer->Lhead[i];
            temp_vel = temp_vel * cos(init_path[0][2+i] - init_path[0][3+i])
                        + temp_w * trailer->Ltail[i] *sin(init_path[0][2+i] - init_path[0][3+i]);
            temp_w = init_theta.col(1)[i];
        }

        // compute length and set init time
        double total_len = 0.0;
        for (size_t k=0; k<init_path.size()-1; k++)
        {
            double temp_seg = (init_path[k+1]-init_path[k]).head(2).norm();
            total_len += temp_seg;
        }
        if (piece_num == 0)
            piece_num_pos = max((int) (total_len / piece_len), 2);
        else
            piece_num_pos = piece_num;
        std::cout<<"piece_num_pos = "<<piece_num_pos<<std::endl;
        piece_num_theta = piece_times * piece_num_pos;

        // initialize variables
        double total_time = getDurationTrapezoid(total_len, temp_vel, 0.0, max_vel, max_alon);
        // double total_time = getDurationTrapezoid(total_len, init_vel, 0.0, max_vel, max_alon);
        double time_per_piece = total_time / piece_num_pos;
        Eigen::MatrixXd inner_pos = Eigen::MatrixXd::Zero(2, piece_num_pos-1);
        Eigen::MatrixXd inner_theta = Eigen::MatrixXd::Zero(TRAILER_NUM, piece_num_theta-1);
        std::vector<Eigen::VectorXd> constrain_points;
        Eigen::VectorXd temp_node;
        /// initialize pos
        for (int i=0; i<piece_num_pos; i++)
        {
            for (int j=1; j<=int_K; j++)
            {
                Eigen::VectorXd temp_node_se2;
                double t = 1.0 * i / piece_num_pos * total_time + 1.0 * j / int_K * time_per_piece;
                double arc = getArcTrapezoid(t, total_len, temp_vel, 0.0, max_vel, max_alon);
                // double arc = getArcTrapezoid(t, total_len, init_vel, 0.0, max_vel, max_alon);

                double temp_len = 0.0;
                for (size_t k=0; k<init_path.size()-1; k++)
                {
                    double temp_seg = (init_path[k+1]-init_path[k]).head(2).norm();
                    temp_len += temp_seg;
                    if (temp_len >= arc)
                    {
                        temp_node = init_path[k] + (1.0 - (temp_len-arc) / temp_seg) * (init_path[k+1] - init_path[k]);
                        break;
                    }
                }
                trailer->gainSE2State(temp_node, temp_node_se2);
                constrain_points.push_back(temp_node_se2);
                if (j == int_K && i != piece_num_pos-1)
                {
                    inner_pos.col(i) = temp_node.head(2);
                }
            }
        }
        /// initialize theta
        for (int i=0; i<piece_num_theta-1; i++)
        {
            double t = 1.0 * (i+1) / piece_num_theta * total_time;
            double arc = getArcTrapezoid(t, total_len, 0.0, 0.0, max_vel, max_alon);
            // double arc = getArcTrapezoid(t, total_len, init_vel, 0.0, max_vel, max_alon);
            double temp_len = 0.0;
            for (size_t k=0; k<init_path.size()-1; k++)
            {
                double temp_seg = (init_path[k+1]-init_path[k]).head(2).norm();
                temp_len += temp_seg;
                if (temp_len > arc)
                {
                    Eigen::VectorXd temp_theta = init_path[k].tail(TRAILER_NUM) + (1.0 - (temp_len-arc) / temp_seg) * (init_path[k+1] - init_path[k]).tail(TRAILER_NUM);
                    inner_theta.col(i) = temp_theta;
                    break;
                }
            } 
        }

        // optimize
        piece_num_pos = inner_pos.cols() + 1;
        piece_num_theta = inner_theta.cols() + 1;
        arc_opt.reset(piece_num_pos);
        head_opt.reset(piece_num_pos);
        tails_opt.reset(piece_num_theta);

        int variable_num = 2*(piece_num_pos-1) + TRAILER_NUM*(piece_num_theta-1) + 1 + piece_num_pos;

        // trailer kinetics
        equal_num = piece_num_pos * int_K * TRAILER_NUM;
        // longitude velocity, min lon vel, longitude acceleration, latitude acceleration, curvature, s'>0, theta_diff
        non_equal_num = piece_num_pos * int_K * 6;
        // non_equal_num = piece_num_pos * (int_K + 1) * (4 + TRAILER_NUM*2);
        if (collision_type == ESDF)
        {
            non_equal_num += piece_num_pos * int_K * (TRAILER_NUM + 3);
            // non_equal_num += piece_num_pos * int_K * (TRAILER_NUM + 1);
        }

        // tail variables
        {
            variable_num += TRAILER_NUM + 3;
            non_equal_num += (TRAILER_NUM + 1) * 4 * 4;
        }

        hx.resize(equal_num);
        hx.setZero();
        lambda.resize(equal_num);
        lambda.setZero();
        gx.resize(non_equal_num);
        gx.setZero();
        mu.resize(non_equal_num);
        mu.setZero();
        scale_fx = 1.0;
        scale_cx.resize(equal_num+non_equal_num);
        scale_cx.setConstant(1.0);
        rho = rho_init;

        std::cout<<"variable_num = "<<variable_num<<std::endl;

        // init solution
        Eigen::VectorXd x;
        x.resize(variable_num);

        dim_time = 1;
        double& tau = x(0);
        int opt_var_idx = dim_time;
        Eigen::Map<Eigen::MatrixXd> Ppos(x.data()+opt_var_idx, 2, piece_num_pos-1);
        opt_var_idx += 2*(piece_num_pos-1);
        Eigen::Map<Eigen::MatrixXd> Ptheta(x.data()+opt_var_idx, TRAILER_NUM, piece_num_theta-1);
        opt_var_idx += TRAILER_NUM*(piece_num_theta-1);
        Eigen::Map<Eigen::VectorXd> Varc(x.data()+opt_var_idx, piece_num_pos);
        opt_var_idx += piece_num_pos;

        tau = logC2(total_time);
        Ppos = inner_pos;
        Ptheta = inner_theta;

        // tail variables
        {
            Eigen::Map<Eigen::VectorXd> Vphi(x.data()+opt_var_idx, TRAILER_NUM);
            Vphi.setZero();
            opt_var_idx += TRAILER_NUM;
            Eigen::Map<Eigen::VectorXd> Tailtractor(x.data()+opt_var_idx, 3);
            opt_var_idx += 3;
            Tailtractor.head(2) = end_pos.col(0);
            Tailtractor(2) = init_path.back()[2];
            // PRINT_GREEN("Tailtractor: ");
            // PRINT_GREEN(Tailtractor.transpose());
        }
        
        for (int i=0; i<piece_num_pos; i++)
        {
            if (i == 0)
                Varc(i) = logC2((inner_pos.col(i) - init_pos.col(0)).norm());
            else if (i == piece_num_pos-1)
                Varc(i) = logC2((inner_pos.col(i-1) - end_pos.col(0)).norm());
            else
                Varc(i) = logC2((inner_pos.col(i) - inner_pos.col(i-1)).norm());
        }
        Eigen::VectorXd Tarc, Ttheta, Arc;
        Tarc.resize(piece_num_pos);
        Arc.resize(piece_num_pos);
        calTfromTauUni(tau, Tarc);
        calTfromTau(Varc, Arc);
        Ttheta.resize(piece_num_theta);
        calTfromTauUni(tau, Ttheta);
        Eigen::MatrixXd init_arc, end_arc;
        init_arc.resize(1, 3);
        end_arc.resize(1, 3);
        init_arc << 0.0, start_v, 0.0;
        end_arc << Arc.sum(), 0.0, 0.0;
        Eigen::MatrixXd Parc;
        Parc.resize(1, piece_num_pos-1);
        for (int i=0; i<piece_num_pos-1; i++)
        {
            if (i==0)
                Parc(0, i) = Arc(i);
            else
                Parc(0, i) = Arc(i) + Parc(0, i-1);
        }
        arc_opt.generate(init_arc, end_arc, Parc, Tarc);
        head_opt.generate(init_pos, end_pos, Ppos, Arc);
        tails_opt.generate(init_theta, end_theta, Ptheta, Ttheta);

        // pubDebugTraj(getTraj());
        // PRINT_GREEN("[Arc Optimizer] Before Optimization:");
        // printConstraintsSituations(getTraj());

        // lbfgs params
        lbfgs::lbfgs_parameter_t lbfgs_params;
        lbfgs_params.mem_size = mem_size;
        lbfgs_params.past = past;
        lbfgs_params.g_epsilon = g_epsilon;
        lbfgs_params.min_step = min_step;
        lbfgs_params.delta = delta;
        lbfgs_params.max_iterations = inner_max_iter;
        double inner_cost;

        // begin PHR-ALM Method
        ros::Time start_time = ros::Time::now();
        int iter = 0;
        bool success = false;
        while (true)
        {
            int result = lbfgs::lbfgs_optimize(x, inner_cost, &innerCallback, nullptr, 
                                            &earlyExit, this, lbfgs_params);

            if (result == lbfgs::LBFGS_CONVERGENCE ||
                result == lbfgs::LBFGS_CANCELED ||
                result == lbfgs::LBFGS_STOP || 
                result == lbfgs::LBFGSERR_MAXIMUMITERATION)
            {
                ;
                // PRINTF_WHITE("[ALM Inner] iters now = "+to_string(iter+1)+"\n");
                // PRINTF_WHITE("[ALM Inner] optimization success! return = "+to_string(result)+" cost: "+to_string(inner_cost)+"\n");
                // PRINTF_WHITE("[ALM Inner] time consuming = "+to_string((double)(clock()-t0)/1000.0)+" ms\n");
            }
            else if (result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH)
            {
                ;
                // PRINT_YELLOW("[ALM Inner] The line-search routine reaches the maximum number of evaluations.");
            }
            else
            {
                ;
                PRINT_RED("[ALM Inner] Solver error. Return = "+to_string(result)+", "+lbfgs::lbfgs_strerror(result)+".");
                success = false;
                break;
            }

            updateDualVars();

            // if (in_debug)
            // {
                // pubDebugTraj(getTraj());
                // std::this_thread::sleep_for(std::chrono::milliseconds(10));
            // }

            if(judgeConvergence())
            {
                success = true;
                PRINTF_WHITE("[ALM] Convergence! iters: "+to_string(iter+1)+"\n");
                break;
            }

            if(++iter == max_iter)
            {
                success = false;
                PRINT_YELLOW("[ALM] Reach max iteration");
                break;
            }
        }
        double opt_time = (ros::Time::now()-start_time).toSec() * 1000.0;
        PRINTF_WHITE("[ALM] Time consuming: "+to_string(opt_time)+" ms\n");
        planning_time = opt_time;
        
        ArcTraj traj = getTraj();

        return success;
    }

    double ArcOpt::innerCallback(void* ptrObj, const Eigen::VectorXd& x, Eigen::VectorXd& grad)
    {
        // PRINT_GREEN(x.transpose());
        ArcOpt& obj = *(ArcOpt*)ptrObj;
        double cost = 0.0;

        // get x
        const double& tau = x(0);
        double& grad_tau = grad(0);
        int opt_var_idx = obj.dim_time;

        Eigen::Map<const Eigen::MatrixXd> Ppos(x.data() + opt_var_idx, 2, obj.piece_num_pos - 1);
        Eigen::Map<Eigen::MatrixXd> gradPos(grad.data() + opt_var_idx, 2, obj.piece_num_pos - 1);
        opt_var_idx += 2*(obj.piece_num_pos-1);

        Eigen::Map<const Eigen::MatrixXd> Ptheta(x.data()+opt_var_idx, TRAILER_NUM, obj.piece_num_theta-1);
        Eigen::Map<Eigen::MatrixXd> gradTheta(grad.data()+opt_var_idx, TRAILER_NUM, obj.piece_num_theta-1);
        opt_var_idx += TRAILER_NUM * (obj.piece_num_theta-1);

        Eigen::Map<const Eigen::VectorXd> Varc(x.data()+opt_var_idx, obj.piece_num_pos);
        Eigen::Map<Eigen::VectorXd> gradVarc(grad.data()+opt_var_idx, obj.piece_num_pos);
        opt_var_idx += obj.piece_num_pos;

        Eigen::Map<const Eigen::VectorXd> Vphi(x.data()+opt_var_idx, TRAILER_NUM);
        Eigen::Map<Eigen::VectorXd> gradVphi(grad.data()+opt_var_idx, TRAILER_NUM);
        opt_var_idx += TRAILER_NUM;
        
        Eigen::Map<const Eigen::VectorXd> Tailtractor(x.data()+opt_var_idx, 3);
        Eigen::Map<Eigen::VectorXd> gradTailtractor(grad.data()+opt_var_idx, 3);
        opt_var_idx += 3;
        
        // get T from τ, generate MINCO trajectory
        Eigen::VectorXd Tarc, Ttheta, dtheta_end, Arc;
        Tarc.resize(obj.piece_num_pos);
        Arc.resize(obj.piece_num_pos);
        obj.calTfromTauUni(tau, Tarc);
        obj.calTfromTau(Varc, Arc);
        Ttheta.resize(obj.piece_num_theta);
        obj.calTfromTauUni(tau, Ttheta);
        dtheta_end.resize(TRAILER_NUM);
        Eigen::MatrixXd init_arc, end_arc;
        init_arc.resize(1, 3);
        end_arc.resize(1, 3);
        init_arc << 0.0, obj.start_v, 0.0;
        end_arc << Arc.sum(), 0.0, 0.0;
        Eigen::MatrixXd Parc;
        Parc.resize(1, obj.piece_num_pos-1);
        for (int i=0; i<obj.piece_num_pos-1; i++)
        {
            if (i==0)
                Parc(0, i) = Arc(i);
            else
                Parc(0, i) = Arc(i) + Parc(0, i-1);
        }

        obj.arc_opt.generate(init_arc, end_arc, Parc, Tarc);

        Eigen::MatrixXd trailer_end;    trailer_end.resize(TRAILER_NUM, 3);    trailer_end.setZero();
        double theta_last = Tailtractor[2];
        for (size_t i=0; i<TRAILER_NUM; i++)
        {
            dtheta_end(i) = obj.sigmoidC2(Vphi(i));
            trailer_end.col(0)[i] = theta_last - dtheta_end(i);
            while (obj.end_theta.col(0)[i] - trailer_end.col(0)[i] >= M_PI_2)
            {
                trailer_end.col(0)[i] += M_PI * 2.0;
            }
            while (obj.end_theta.col(0)[i] - trailer_end.col(0)[i] <= -M_PI_2)
            {
                trailer_end.col(0)[i] -= M_PI * 2.0;
            }
            theta_last = trailer_end.col(0)[i];
        }

        Eigen::MatrixXd tractor_end = obj.end_pos;
        tractor_end.col(0) = Tailtractor.head(2);
        tractor_end.col(1)[0] = cos(Tailtractor[2]);
        tractor_end.col(1)[1] = sin(Tailtractor[2]);
        obj.head_opt.generate(obj.init_pos, tractor_end, Ppos, Arc);
        obj.tails_opt.generate(obj.init_theta, trailer_end, Ptheta, Ttheta);

        // get jerk cost with grad (C,T)
        double jerk_cost = 0.0;
        Eigen::MatrixXd gdCpos_jerk, gdCtheta_jerk, gdCarc_jerk;
        Eigen::VectorXd gdApos_jerk, gdTtheta_jerk, gdTarc_jerk;
        
        // jerk
        obj.head_opt.calJerkGradCT(gdCpos_jerk, gdApos_jerk);
        obj.tails_opt.calJerkGradCT(gdCtheta_jerk, gdTtheta_jerk);
        obj.arc_opt.calJerkGradCT(gdCarc_jerk, gdTarc_jerk);
        jerk_cost = obj.head_opt.getTrajJerkCost() \
                    + obj.tails_opt.getTrajJerkCost() \
                    + obj.arc_opt.getTrajJerkCost();

        jerk_cost *= obj.inner_weight_jerk;
        gdCpos_jerk *= obj.inner_weight_jerk;
        gdApos_jerk *= obj.inner_weight_jerk;
        gdCtheta_jerk *= obj.inner_weight_jerk;
        gdTtheta_jerk *= obj.inner_weight_jerk;
        gdCarc_jerk *= obj.inner_weight_jerk;
        gdTarc_jerk *= obj.inner_weight_jerk;

        // get constrain cost with grad (C,T)
        double constrain_cost = 0.0;
        Eigen::MatrixXd gdCpos_constrain, gdCtheta_constrain, gdCarc_constrain;
        Eigen::VectorXd gdApos_constrain, gdTtheta_constrain, gdTarc_constrain;
        obj.calConstrainCostGrad(constrain_cost, gdCpos_constrain, gdApos_constrain, 
                                                gdCtheta_constrain, gdTtheta_constrain,
                                                    gdCarc_constrain, gdTarc_constrain);
        // std::cout<<"constrain_cost = "<<constrain_cost<<std::endl;

        // get grad (q, T) from (C, T)
        Eigen::MatrixXd gdCpos = gdCpos_jerk + gdCpos_constrain;
        Eigen::VectorXd gdApos = gdApos_jerk + gdApos_constrain;
        Eigen::MatrixXd gdCtheta = gdCtheta_jerk + gdCtheta_constrain;
        Eigen::VectorXd gdTtheta = gdTtheta_jerk + gdTtheta_constrain;
        Eigen::MatrixXd gdCarc = gdCarc_jerk + gdCarc_constrain;
        Eigen::VectorXd gdTarc = gdTarc_jerk + gdTarc_constrain;
        Eigen::MatrixXd gradPpos_temp, gradPtheta_temp, gradParc;
        Eigen::MatrixXd gradPtail_tractor, gradPtail_trailer, gradPtail_arc;
        obj.head_opt.calGradCTtoQT(gdCpos, gdApos, gradPpos_temp, gradPtail_tractor);
        obj.tails_opt.calGradCTtoQT(gdCtheta, gdTtheta, gradPtheta_temp, gradPtail_trailer);
        obj.arc_opt.calGradCTtoQT(gdCarc, gdTarc, gradParc, gradPtail_arc);
        gradPos = gradPpos_temp;
        gradTheta = gradPtheta_temp;
        // gradVarc from Parc
        Eigen::VectorXd gradArc_temp = Eigen::VectorXd::Zero(obj.piece_num_pos);
        gradArc_temp(obj.piece_num_pos-1) = gradPtail_arc(0, 0);
        for (int i=obj.piece_num_pos-2; i>=0; i--)
        {
            gradArc_temp(i) = gradArc_temp(i+1) + gradParc(0, i);
        }
        // gradVarc from Apos and temp
        gradVarc.setZero();
        for (int i=0; i<obj.piece_num_pos; i++)
        {
            gradVarc(i) += (gdApos(i) + gradArc_temp(i)) * obj.getTtoTauGrad(Varc(i));
        }

        double tail_cost = 0.0;

        // tail variables
        {
            // tail state
            Eigen::VectorXd gdTail_temp;
            Eigen::VectorXd gdtailTheta_temp;
            Eigen::Vector3d Tailtractor_temp = Tailtractor;
            Eigen::Vector3d gradTailtractor_temp;
            obj.calTailStateCostGrad(tail_cost, gdtailTheta_temp, dtheta_end,
                                    Tailtractor_temp, gradTailtractor_temp);
            
            // from gdPtail to gdTail
            gdtailTheta_temp += gradPtail_trailer.col(0);
            gdTail_temp.resize(TRAILER_NUM);
            gdTail_temp.setZero();
            for (int i=TRAILER_NUM-1; i>=0; i--)
            {
                gdTail_temp(i) -= gdtailTheta_temp(i);
                if (i > 0)
                    gdtailTheta_temp(i-1) += gdtailTheta_temp(i);
            }
            gradTailtractor_temp.head(2) += gradPtail_tractor.col(0);
            gradTailtractor_temp[2] -= gdTail_temp(0);

            gradVphi.setZero();
            for (size_t i=0; i<TRAILER_NUM; i++)
                gradVphi(i) += gdTail_temp(i) * obj.getThetatoVphiGrad(Vphi(i));
            
            gradTailtractor = gradTailtractor_temp;
        }

        // get tau cost with grad
        double tau_cost = obj.rho_T * obj.expC2(tau);
        double grad_Tsum = obj.rho_T + gdTarc.sum() / obj.piece_num_pos + gdTtheta.sum() / obj.piece_num_theta;
        grad_tau = grad_Tsum * obj.getTtoTauGrad(tau);

        cost = jerk_cost + constrain_cost + tau_cost + tail_cost;

        return cost;
    }

    void ArcOpt::calConstrainCostGrad(double& cost, Eigen::MatrixXd& gdCpos, Eigen::VectorXd &gdApos,
                                                    Eigen::MatrixXd& gdCtheta, Eigen::VectorXd &gdTtheta,
                                                    Eigen::MatrixXd& gdCarc, Eigen::VectorXd &gdTarc)
    {
        cost = 0.0;
        gdCpos.resize(6*piece_num_pos, 2);
        gdCpos.setZero();
        gdCtheta.resize(6*piece_num_theta, TRAILER_NUM);
        gdCtheta.setZero();
        gdApos.resize(piece_num_pos);
        gdApos.setZero();
        gdCarc.resize(6*piece_num_pos, 1);
        gdCarc.setZero();
        gdTarc.resize(piece_num_pos);
        gdTarc.setZero();
        gdTtheta.resize(piece_num_theta);
        gdTtheta.setZero();

        Eigen::Vector2d pos, vel, acc, jer;
        Eigen::Vector2d real_vel;
        double arc, darc, d2arc, d3arc;
        Eigen::VectorXd theta, dtheta, d2theta, d3theta;

        double grad_arc = 0.0;
        double grad_darc = 0.0;
        double grad_d2arc = 0.0;

        Eigen::Vector2d grad_p = Eigen::Vector2d::Zero();
        Eigen::Vector2d grad_v = Eigen::Vector2d::Zero();
        Eigen::Vector2d grad_a = Eigen::Vector2d::Zero();

        Eigen::VectorXd grad_theta;
        grad_theta.resize(TRAILER_NUM); grad_theta.setZero();
        Eigen::VectorXd grad_dtheta;
        grad_dtheta.resize(TRAILER_NUM); grad_dtheta.setZero();
        Eigen::VectorXd grad_d2theta;
        grad_d2theta.resize(TRAILER_NUM); grad_d2theta.setZero();
        Eigen::VectorXd vtrailer, grad_vtrailer;
        vtrailer.resize(TRAILER_NUM+1); vtrailer.setZero();
        grad_vtrailer.resize(TRAILER_NUM); grad_vtrailer.setZero();
        Eigen::MatrixXd ptrailer, grad_ptrailer;
        ptrailer.resize(2, TRAILER_NUM+1); ptrailer.setZero();
        grad_ptrailer.resize(2, TRAILER_NUM+1); grad_ptrailer.setZero();

        Eigen::Matrix<double, 6, 1> beta0_pos, beta1_pos, beta2_pos, beta3_pos, beta4_pos;
        Eigen::Matrix<double, 6, 1> beta0_arc, beta1_arc, beta2_arc, beta3_arc, beta4_arc;
        Eigen::Matrix<double, 6, 1> beta0_theta, beta1_theta, beta2_theta;

        double s1_pos, s2_pos, s3_pos, s4_pos, s5_pos;
        double s1_arc, s2_arc, s3_arc, s4_arc, s5_arc;
        double s1_theta, s2_theta, s3_theta, s4_theta, s5_theta;
        double step, alpha;

        int equal_idx = 0;
        int non_equal_idx = 0;
        int constrain_idx = 0;
        int theta_idx = 0;
        double aug_grad = 0.0;
        double base_time = 0.0;
        double base_arc = 0.0;

        double theta0, vlon, inv_vlon, inv_vlon2, alon, alat, curv, curv_snorm, dtheta0;
        Eigen::VectorXd theta_diff, sthetad, cthetad;
        theta_diff.resize(TRAILER_NUM);
        sthetad.resize(TRAILER_NUM);
        cthetad.resize(TRAILER_NUM);

        int corridor_idx = 0;
        for (int i=0; i<piece_num_pos; i++)
        {
            const Eigen::Matrix<double, 6, 1> &c = arc_opt.getCoeffs().block<6, 1>(i * 6, 0);
            const Eigen::Matrix<double, 6, 2> &c_pos = head_opt.getCoeffs().block<6, 2>(i * 6, 0);
            step = arc_opt.T1(i) / int_K;
            s1_arc = step;

            for (int j=1; j<=int_K; j++)
            {
                clock_t t1 = clock();
                alpha = 1.0 / int_K * j;
                // double omg = (j == 0 || j == int_K) ? 0.5 : 1.0;

                // set zero
                grad_arc = 0.0;
                grad_darc = 0.0;
                grad_d2arc = 0.0;
                grad_p.setZero();
                grad_v.setZero();
                grad_a.setZero();
                grad_theta.setZero();
                grad_dtheta.setZero();
                grad_d2theta.setZero();
                grad_vtrailer.setZero();
                grad_ptrailer.setZero();
                double grad_theta0 = 0.0;
                double grad_dtheta0 = 0.0;

                // analyse arc
                s2_arc = s1_arc * s1_arc;
                s3_arc = s2_arc * s1_arc;
                s4_arc = s2_arc * s2_arc;
                s5_arc = s2_arc * s3_arc;
                beta0_arc << 1.0, s1_arc, s2_arc, s3_arc, s4_arc, s5_arc;
                beta1_arc << 0.0, 1.0, 2.0 * s1_arc, 3.0 * s2_arc, 4.0 * s3_arc, 5.0 * s4_arc;
                beta2_arc << 0.0, 0.0, 2.0, 6.0 * s1_arc, 12.0 * s2_arc, 20.0 * s3_arc;
                beta3_arc << 0.0, 0.0, 0.0, 6.0, 24.0 * s1_arc, 60.0 * s2_arc;
                arc = c.transpose() * beta0_arc;
                darc = c.transpose() * beta1_arc;
                d2arc = c.transpose() * beta2_arc;
                d3arc = c.transpose() * beta3_arc;

                // analyse xy
                s1_pos = arc - base_arc;
                s2_pos = s1_pos * s1_pos;
                s3_pos = s2_pos * s1_pos;
                s4_pos = s2_pos * s2_pos;
                s5_pos = s2_pos * s3_pos;
                beta0_pos << 1.0, s1_pos, s2_pos, s3_pos, s4_pos, s5_pos;
                beta1_pos << 0.0, 1.0, 2.0 * s1_pos, 3.0 * s2_pos, 4.0 * s3_pos, 5.0 * s4_pos;
                beta2_pos << 0.0, 0.0, 2.0, 6.0 * s1_pos, 12.0 * s2_pos, 20.0 * s3_pos;
                beta3_pos << 0.0, 0.0, 0.0, 6.0, 24.0 * s1_pos, 60.0 * s2_pos;
                pos = c_pos.transpose() * beta0_pos;
                vel = c_pos.transpose() * beta1_pos;
                acc = c_pos.transpose() * beta2_pos;
                jer = c_pos.transpose() * beta3_pos;
                real_vel = vel * darc;

                // analyse theta
                double now_time = s1_arc + base_time;
                theta_idx = int((now_time) / tails_opt.T1(i));
                if (theta_idx >= piece_num_theta)
                    theta_idx = piece_num_theta - 1;
                const Eigen::MatrixXd theta_coeffs = tails_opt.getCoeffs();
                const Eigen::Matrix<double, 6, TRAILER_NUM>& c_theta = theta_coeffs.block<6, TRAILER_NUM>(theta_idx * 6, 0);
                s1_theta = now_time - theta_idx * tails_opt.T1(i);
                s2_theta = s1_theta * s1_theta;
                s3_theta = s2_theta * s1_theta;
                s4_theta = s2_theta * s2_theta;
                s5_theta = s4_theta * s1_theta;
                beta0_theta << 1.0, s1_theta, s2_theta, s3_theta, s4_theta, s5_theta;
                beta1_theta << 0.0, 1.0, 2.0 * s1_theta, 3.0 * s2_theta, 4.0 * s3_theta, 5.0 * s4_theta;
                beta2_theta << 0.0, 0.0, 2.0, 6.0 * s1_theta, 12.0 * s2_theta, 20.0 * s3_theta;
                theta = c_theta.transpose() * beta0_theta;
                dtheta = c_theta.transpose() * beta1_theta;
                d2theta = c_theta.transpose() * beta2_theta;

                // analyse flatness variables
                theta0 = atan2(vel(1), vel(0));
                vlon = vel.norm();
                inv_vlon = 1.0 / vlon;
                inv_vlon2 = inv_vlon * inv_vlon;
                alon = vlon*d2arc + vel.dot(acc) * inv_vlon * darc * darc;
                alat = (acc(1)*vel(0) - acc(0)*vel(1)) * inv_vlon * darc * darc;
                curv = (acc(1)*vel(0) - acc(0)*vel(1)) * inv_vlon * inv_vlon2;
                curv_snorm = curv * curv;
                dtheta0 = darc * (acc(1)*vel(0) - acc(0)*vel(1)) * inv_vlon2;

                theta_diff[0] = theta0 - theta[0];
                sthetad[0] = sin(theta_diff[0]);
                cthetad[0] = cos(theta_diff[0]);
                for (size_t i=0; i<TRAILER_NUM-1; i++)
                {
                    theta_diff[i+1] = theta[i] - theta[i+1];
                    sthetad[i+1] = sin(theta_diff[i+1]);
                    cthetad[i+1] = cos(theta_diff[i+1]);
                }

                // trailer kinetics
                vtrailer[0] = vlon * darc;
                for (size_t k=0; k<TRAILER_NUM; k++)
                {
                    double kinetics_lambda = lambda[equal_idx];
                    // hx[equal_idx] = (vtrailer[k]*sthetad[k] - trailer->Lhead[k]*dtheta[k]) * 100.0;
                    // hx[equal_idx] = (vtrailer[k]*sthetad[k] - trailer->Lhead[k]*dtheta[k]) * scale_cx(constrain_idx);
                    double temp_dth = k > 0? dtheta[k-1] : dtheta0;
                    hx[equal_idx] = (vtrailer[k]*sthetad[k] - trailer->Lhead[k]*dtheta[k] 
                                    - trailer->Ltail[k] * temp_dth * cthetad[k]) * scale_cx(constrain_idx);
                    double kinetics_cost = getAugmentedCost(hx[equal_idx], kinetics_lambda) * inner_weight_kinetics;
                    cost += kinetics_cost;

                    // double kinetics_grad = getAugmentedGrad(hx[equal_idx], kinetics_lambda) * 100.0;
                    double kinetics_grad = getAugmentedGrad(hx[equal_idx], kinetics_lambda) * scale_cx(constrain_idx) * inner_weight_kinetics;
                    grad_vtrailer[k] = kinetics_grad * sthetad[k];
                    if (k>0)
                    {
                        grad_theta[k-1] += kinetics_grad * (vtrailer[k] * cthetad[k]
                                            +trailer->Ltail[k] * temp_dth * sthetad[k]);
                        grad_dtheta[k-1] -= kinetics_grad * trailer->Ltail[k] * cthetad[k];
                    }
                    else
                    {
                        grad_theta0 += kinetics_grad * (vtrailer[k] * cthetad[k]
                                        +trailer->Ltail[k] * temp_dth * sthetad[k]);
                        grad_dtheta0 -= kinetics_grad * trailer->Ltail[k] * cthetad[k];
                    }
                        
                    grad_theta[k] -= kinetics_grad * (vtrailer[k] * cthetad[k]
                                    +trailer->Ltail[k] * temp_dth * sthetad[k]);
                    grad_dtheta[k] -= kinetics_grad * trailer->Lhead[k];
                    equal_idx++;
                    constrain_idx++;
                    vtrailer[k+1] = vtrailer[k] * cthetad[k] + trailer->Ltail[k] * temp_dth * sthetad[k];
                }
                for (int k=TRAILER_NUM-1; k>=0; k--)
                {
                    if (k==0)
                    {
                        grad_v += grad_vtrailer[k] * inv_vlon * darc * vel;
                        grad_darc += grad_vtrailer[k] * vlon;
                    }
                    else if (k==1)
                    {
                        grad_dtheta0 += grad_vtrailer[k] * sthetad[k-1] * trailer->Ltail[k-1];
                        grad_theta0 -= grad_vtrailer[k] * (sthetad[k-1] * vtrailer[k-1]
                                                                - trailer->Ltail[k-1] * dtheta0 * cthetad[k-1]);
                        grad_theta[k-1] += grad_vtrailer[k] * sthetad[k-1] * vtrailer[k-1];
                        grad_vtrailer[k-1] += grad_vtrailer[k] * cthetad[k-1];

                    }
                    else
                    {
                        grad_dtheta[k-2] += grad_vtrailer[k] * sthetad[k-1] * trailer->Ltail[k-1];
                        grad_theta[k-2] -= grad_vtrailer[k] * (sthetad[k-1] * vtrailer[k-1]
                                                                - trailer->Ltail[k-1] * dtheta[k-2] * cthetad[k-1]);
                        grad_theta[k-1] += grad_vtrailer[k] * (sthetad[k-1] * vtrailer[k-1]
                                                                - trailer->Ltail[k-1] * dtheta[k-2] * cthetad[k-1]);
                        grad_vtrailer[k-1] += grad_vtrailer[k] * cthetad[k-1];
                    }
                }

                // darc > 0
                double arc_mu = mu[non_equal_idx];
                double arc_cost = 0.0;
                gx[non_equal_idx] = -darc * scale_cx(constrain_idx);
                // std::cout<<"cost darc = "<<darc<<std::endl;
                if (rho * gx[non_equal_idx] + arc_mu > 0)
                {
                    arc_cost = getAugmentedCost(gx[non_equal_idx], arc_mu);
                    aug_grad = getAugmentedGrad(gx[non_equal_idx], arc_mu) * scale_cx(constrain_idx);
                    grad_darc -= aug_grad;
                }
                else
                {
                    arc_cost = -0.5 * arc_mu * arc_mu / rho;
                }
                cost += arc_cost;
                // std::cout<<"arc_cost"<<arc_cost<<std::endl;
                non_equal_idx++;
                constrain_idx++;
                
                // longitude velocity
                //realVel (vel * ds)
                double v_mu = mu[non_equal_idx];
                double vel_cost = 0.0;
                gx[non_equal_idx] = (real_vel.squaredNorm() - max_vel*max_vel) * scale_cx(constrain_idx);
                if (rho * gx[non_equal_idx] + v_mu > 0)
                {
                    vel_cost = getAugmentedCost(gx[non_equal_idx], v_mu) * inner_weight_vel;
                    aug_grad = getAugmentedGrad(gx[non_equal_idx], v_mu) * scale_cx(constrain_idx) * inner_weight_vel;
                    grad_v += aug_grad * 2.0 * vel * darc * darc;
                    grad_darc += aug_grad * 2.0 * darc * vel.squaredNorm();
                }
                else
                {
                    vel_cost = -0.5 * v_mu * v_mu / rho * inner_weight_vel;
                    
                }
                cost += vel_cost;
                // std::cout<<"vel_cost"<<vel_cost<<std::endl;
                non_equal_idx++;
                constrain_idx++;

                // min pseudo longitude velocity
                double vmin_mu = mu[non_equal_idx];
                double min_vel_cost = 0.0;
                gx[non_equal_idx] = (0.01 - vel.squaredNorm()) * scale_cx(constrain_idx);
                if (rho * gx[non_equal_idx] + vmin_mu > 0)
                {
                    min_vel_cost = getAugmentedCost(gx[non_equal_idx], vmin_mu) * inner_weight_min_vel;
                    aug_grad = getAugmentedGrad(gx[non_equal_idx], vmin_mu) * scale_cx(constrain_idx) * inner_weight_min_vel;
                    grad_v -= aug_grad * 2.0 * vel;
                }
                else
                {
                    min_vel_cost = -0.5 * vmin_mu * vmin_mu / rho * inner_weight_min_vel;
                }
                cost += min_vel_cost;
                // std::cout<<"min_vel_cost"<<min_vel_cost<<std::endl;
                non_equal_idx++;
                constrain_idx++;

                // longitude acceleration
                // alon = vlon*d2arc + vel.dot(acc) * inv_vlon * darc * darc;
                double lona_mu = mu[non_equal_idx];
                double alon_cost = 0.0;
                gx[non_equal_idx] = (alon*alon - max_alon*max_alon) * scale_cx(constrain_idx);
                if (rho * gx[non_equal_idx] + lona_mu > 0)
                {
                    double darc2 = darc * darc;
                    alon_cost = getAugmentedCost(gx[non_equal_idx], lona_mu) * inner_weight_alon;
                    aug_grad = getAugmentedGrad(gx[non_equal_idx], lona_mu) * scale_cx(constrain_idx) * inner_weight_alon;
                    grad_a += aug_grad * 2.0 * alon * inv_vlon * darc2 * vel;
                    grad_v += aug_grad * 2.0 * alon * (inv_vlon * darc2 * acc + inv_vlon * d2arc * vel);
                    grad_v -= aug_grad * 2.0 * alon * vel.dot(acc) * inv_vlon2 * inv_vlon * darc2 * vel;
                    grad_d2arc += aug_grad * 2.0 * alon * vlon;
                    grad_darc += aug_grad * 2.0 * alon * 2.0 * vel.dot(acc) * inv_vlon * darc;
                }
                else
                {
                    alon_cost = -0.5 * lona_mu * lona_mu / rho * inner_weight_alon;
                }
                cost += alon_cost;
                // std::cout<<"alon_cost"<<alon_cost<<std::endl;
                non_equal_idx++;
                constrain_idx++;

                // latitude acceleration
                // alat = (acc(1)*vel(0) - acc(0)*vel(1)) * inv_vlon * darc * darc;
                double lata_mu = mu[non_equal_idx];
                double alat_cost = 0.0;
                gx[non_equal_idx] = (alat*alat - max_alat*max_alat) * scale_cx(constrain_idx);
                if (rho * gx[non_equal_idx] + lata_mu > 0)
                {
                    double darc2 = darc * darc;
                    alat_cost = getAugmentedCost(gx[non_equal_idx], lata_mu) * inner_weight_alat;
                    aug_grad = getAugmentedGrad(gx[non_equal_idx], lata_mu) * scale_cx(constrain_idx) * inner_weight_alat;
                    grad_a += aug_grad * 2.0 * alat * inv_vlon * darc2 * Eigen::Vector2d(-vel(1), vel(0));
                    grad_v += aug_grad * 2.0 * alat * inv_vlon * darc2 * Eigen::Vector2d(acc(1), -acc(0));
                    grad_v -= aug_grad * 2.0 * alat * alat * inv_vlon2 * vel;
                    grad_darc += aug_grad * 2.0 * alat * 2.0 * (acc(1)*vel(0) - acc(0)*vel(1)) * inv_vlon * darc;
                }
                else
                {
                    alat_cost = -0.5 * lata_mu * lata_mu / rho * inner_weight_alat;
                }
                cost += alat_cost;
                // std::cout<<"alat_cost"<<alat_cost<<std::endl;
                non_equal_idx++;
                constrain_idx++;

                // curvature
                // curv = (acc(1)*vel(0) - acc(0)*vel(1)) * inv_vlon * inv_vlon2;
                double curv_mu = mu[non_equal_idx];
                double kappa_cost = 0.0;
                gx[non_equal_idx] = (curv_snorm - max_kappa*max_kappa) * scale_cx(constrain_idx);
                if (rho * gx[non_equal_idx] + curv_mu > 0)
                {
                    kappa_cost = getAugmentedCost(gx[non_equal_idx], curv_mu) * inner_weight_kappa;
                    aug_grad = getAugmentedGrad(gx[non_equal_idx], curv_mu) * scale_cx(constrain_idx) * inner_weight_kappa;
                    grad_a += aug_grad * 2.0 * curv * inv_vlon * inv_vlon2 * Eigen::Vector2d(-vel(1), vel(0));
                    grad_v += aug_grad * 2.0 * curv * inv_vlon * inv_vlon2 * Eigen::Vector2d(acc(1), -acc(0));
                    grad_v -= aug_grad * 6.0 * curv_snorm * inv_vlon2 * vel;
                }
                else
                {
                    kappa_cost = -0.5 * curv_mu * curv_mu / rho * inner_weight_kappa;
                }
                cost += kappa_cost;
                // std::cout<<"kappa_cost"<<kappa_cost<<std::endl;
                non_equal_idx++;
                constrain_idx++;

                // collision
                double collision_cost = 0.0;
                if (collision_type == ESDF)
                {
                    double sdf_value;
                    Eigen::Vector2d sdf_grad;

                    std::vector<double> tractor_points; 
                    std::vector<double> tractor_radius; 

                    tractor_points.push_back(0.0);   tractor_radius.push_back(0.8);
                    tractor_points.push_back(0.9);  tractor_radius.push_back(0.8);
                    tractor_points.push_back(1.9);   tractor_radius.push_back(0.8);

                    for (size_t k=0; k<tractor_points.size(); k++)
                    {
                        Eigen::Vector2d penal_point = pos + Eigen::Vector2d(cos(theta0), sin(theta0)) * tractor_points[k];
                        Eigen::Vector2d now_grad_p = Eigen::Vector2d::Zero();
                        grid_map->getDisWithGradI(penal_point, sdf_value, sdf_grad);
                        double colli_mu = mu[non_equal_idx];
                        gx[non_equal_idx] = (tractor_radius[k] - sdf_value) * scale_cx(constrain_idx);
                        if (rho * gx[non_equal_idx] + colli_mu > 0)
                        {
                            collision_cost = getAugmentedCost(gx[non_equal_idx], colli_mu) * inner_weight_collision;
                            aug_grad = getAugmentedGrad(gx[non_equal_idx], colli_mu) * scale_cx(constrain_idx) * inner_weight_collision;
                            now_grad_p = -aug_grad * sdf_grad;
                            grad_ptrailer.col(0) += now_grad_p;
                        }
                        else
                        {
                            collision_cost = -0.5 * colli_mu * colli_mu / rho * inner_weight_collision;
                        }
                        cost += collision_cost;
                        non_equal_idx++;
                        constrain_idx++;
                        grad_theta0 += now_grad_p.dot(Eigen::Vector2d(-sin(theta0), cos(theta0))) * tractor_points[k];
                    }

                    ptrailer.col(0) = pos;
                    ptrailer.col(1) = ptrailer.col(0) - Eigen::Vector2d(cos(theta[0]), sin(theta[0]))*trailer->Lhead[0]
                                        - Eigen::Vector2d(cos(theta0), sin(theta0))*trailer->Ltail[0];
                    for (size_t k=1; k<TRAILER_NUM+1; k++)
                    {
                        grid_map->getDisWithGradI(ptrailer.col(k), sdf_value, sdf_grad);
                        double colli_mu = mu[non_equal_idx];
                        gx[non_equal_idx] = (safe_threshold - sdf_value) * scale_cx(constrain_idx);
                        if (rho * gx[non_equal_idx] + colli_mu > 0)
                        {
                            collision_cost = getAugmentedCost(gx[non_equal_idx], colli_mu) * inner_weight_collision;
                            aug_grad = getAugmentedGrad(gx[non_equal_idx], colli_mu) * scale_cx(constrain_idx) * inner_weight_collision;
                            grad_ptrailer.col(k) = -aug_grad * sdf_grad;
                        }
                        else
                        {
                            collision_cost = -0.5 * colli_mu * colli_mu / rho * inner_weight_collision;
                        }
                        cost += collision_cost;
                        non_equal_idx++;
                        constrain_idx++;
                        if (k<TRAILER_NUM)
                        {
                            ptrailer.col(k+1) = ptrailer.col(k) - Eigen::Vector2d(cos(theta[k]), sin(theta[k]))*trailer->Lhead[k]
                                                - Eigen::Vector2d(cos(theta[k-1]), sin(theta[k-1]))*trailer->Ltail[k];
                        }
                    }
                    for (int k=TRAILER_NUM; k>=0; k--)
                    {
                        if (k==0)
                        {
                            grad_p += grad_ptrailer.col(k);
                        }
                        else if (k==1)
                        {
                            grad_ptrailer.col(k-1) += grad_ptrailer.col(k);
                            grad_theta[k-1] -= grad_ptrailer.col(k).dot(Eigen::Vector2d(-sin(theta[k-1]), cos(theta[k-1])))
                                                                    *trailer->Lhead[k-1];
                            grad_theta0 -= grad_ptrailer.col(k).dot(Eigen::Vector2d(-sin(theta0), cos(theta0)))
                                                                    *trailer->Ltail[k-1];
                        }
                        else
                        {
                            grad_ptrailer.col(k-1) += grad_ptrailer.col(k);
                            grad_theta[k-1] -= grad_ptrailer.col(k).dot(Eigen::Vector2d(-sin(theta[k-1]), cos(theta[k-1])))
                                                                    *trailer->Lhead[k-1];
                            grad_theta[k-2] -= grad_ptrailer.col(k).dot(Eigen::Vector2d(-sin(theta[k-2]), cos(theta[k-2])))
                                                                    *trailer->Ltail[k-1];
                        }
                    }
                }

                grad_v(0) -= grad_theta0 * inv_vlon2 * vel(1);
                grad_v(1) += grad_theta0 * inv_vlon2 * vel(0);

                // NOTO: dtheta0 = darc * (acc(1)*vel(0) - acc(0)*vel(1)) * inv_vlon2;
                grad_darc += grad_dtheta0 * (acc(1)*vel(0) - acc(0)*vel(1)) * inv_vlon2;
                grad_a += grad_dtheta0 * darc * inv_vlon2 * Eigen::Vector2d(-vel(1), vel(0));
                grad_v += grad_dtheta0 * darc * inv_vlon2 * Eigen::Vector2d(acc(1), -acc(0));
                grad_v -= 2.0 * grad_dtheta0 * darc * (acc(1)*vel(0) - acc(0)*vel(1)) * inv_vlon2 * inv_vlon2 * vel;
                
                // add all grad into C,T
                // note that xy = Cxy*β(a-a_last), yaw = Cyaw*β(i*T_xy+j/K*T_xy-theta_idx*T_yaw)
                // ∂p/∂Cxy, ∂v/∂Cxy, ∂a/∂Cxy
                gdCpos.block<6, 2>(i * 6, 0) += (beta0_pos * grad_p.transpose() + \
                                                beta1_pos * grad_v.transpose() + \
                                                beta2_pos * grad_a.transpose());
                // ∂p/∂a
                double grad_arc_temp = (grad_p.dot(vel) + \
                                        grad_v.dot(acc) + \
                                        grad_a.dot(jer) );
                grad_arc += grad_arc_temp;
                if (i>0)
                {
                    gdApos.segment(0, i).array() -= grad_arc_temp;
                }
                
                // note that a = Ca*β(j/K*T_a)
                // ∂p/∂Ca, ∂v/∂Ca, ∂a/∂Ca
                gdCarc.block<6, 1>(i * 6, 0) += (beta0_arc * grad_arc + \
                                                beta1_arc * grad_darc + \
                                                beta2_arc * grad_d2arc);
                // ∂p/∂Txy, ∂v/∂Txy, ∂a/∂Txy
                gdTarc(i) += (grad_arc * darc + \
                            grad_darc * d2arc + \
                            grad_d2arc * d3arc ) * alpha;
                
                // ∂yaw/∂Cyaw, ∂dyaw/∂Cyaw, ∂d2yaw/∂Cyaw
                gdCtheta.block<6, TRAILER_NUM>(theta_idx * 6, 0) += (beta0_theta * grad_theta.transpose() + \
                                                            beta1_theta * grad_dtheta.transpose() + \
                                                            beta2_theta * grad_d2theta.transpose());
                // ∂yaw/∂Tyaw, ∂dyaw/∂Tyaw, ∂d2yaw/∂Tyaw
                gdTtheta(theta_idx) += -(grad_theta.dot(dtheta) +
                                        grad_dtheta.dot(d2theta)) * theta_idx;
                // ∂yaw/∂Txy, ∂dyaw/∂Txy, ∂d2yaw/∂Txy
                gdTarc(i) += (grad_theta.dot(dtheta) +
                            grad_dtheta.dot(d2theta)) * (alpha+i);
                
                s1_arc += step;
            }
            base_time += arc_opt.T1(i);
            base_arc += head_opt.T1(i);
        }
    }

    void ArcOpt::calTailStateCostGrad(double& cost, Eigen::VectorXd& gdTail, const Eigen::VectorXd& dtheta_end,
                                    const Eigen::Vector3d& Tailtractor_temp, Eigen::Vector3d& gradTailtractor_temp)
    {
        cost = 0.0;
        gdTail.resize(TRAILER_NUM);
        gdTail.setZero();
        gradTailtractor_temp.setZero();

        Eigen::VectorXd stheta;
        Eigen::VectorXd ctheta;
        stheta.resize(TRAILER_NUM);
        ctheta.resize(TRAILER_NUM);

        double theta_now = Tailtractor_temp[2];
        // double theta_now = atan2(end_pos.col(1)[1], end_pos.col(1)[0]);
        double ct;
        double st;

        Eigen::Vector2d traileri = Tailtractor_temp.head(2);
        // Eigen::Vector2d traileri = end_pos.col(0);
        
        std::vector<Eigen::Vector2d> bps;
        double h = 0.0;
        double w = trailer->width / 2.0;

        Eigen::Vector2d normal, bp;
        int non_equal_idx = non_equal_num - (TRAILER_NUM+1) * 4 * 4;
        // int non_equal_idx = non_equal_num - TRAILER_NUM * 4 * 4;
        int constrain_idx = equal_num + non_equal_idx;

        // tractor
        {
            ct = cos(theta_now);
            st = sin(theta_now);
            Eigen::Matrix2d car_R, car_R_dot;
            car_R << ct, -st, st, ct;
            car_R_dot << -st, -ct, ct, -st;
            bps.clear();
            bps.push_back(Eigen::Vector2d(trailer->length[0]-trailer->rear_length, w));
            bps.push_back(Eigen::Vector2d(trailer->length[0]-trailer->rear_length, -w));
            bps.push_back(Eigen::Vector2d(-trailer->rear_length, -w));
            bps.push_back(Eigen::Vector2d(-trailer->rear_length, w));
            for (int j=0; j<4; j++)
            {
                for (int i=0; i<4; i++)
                {
                    double tail_cost = 0.0;
                    double tail_mu = mu[non_equal_idx];
                    bp = traileri + car_R * bps[j];
                    normal = trailer->terminal_area.col(i).head(2);
                    gx[non_equal_idx] = normal.dot(bp - trailer->terminal_area.col(i).tail(2)) * scale_cx(constrain_idx);
                    if (rho * gx[non_equal_idx] + tail_mu > 0)
                    {
                        tail_cost = getAugmentedCost(gx[non_equal_idx], tail_mu) * inner_weight_tail;
                        double aug_grad = getAugmentedGrad(gx[non_equal_idx], tail_mu) * scale_cx(constrain_idx)* inner_weight_tail;
                        gradTailtractor_temp[2] += aug_grad * normal.dot(car_R_dot*bps[j]);
                        gradTailtractor_temp.head(2) += aug_grad * normal;
                    }
                    else
                    {
                        tail_cost = -0.5 * tail_mu * tail_mu / rho * inner_weight_tail;
                    }
                    cost += tail_cost;
                    non_equal_idx++;
                    constrain_idx++;
                }
            }
        }
        
        // trailers
        std::vector<Eigen::Vector2d> grad_r2_state;
        for (size_t k=0; k<TRAILER_NUM; k++)
        {
            double ct_pre = ct;
            double st_pre = st;
            grad_r2_state.push_back(Eigen::Vector2d::Zero());
            theta_now = theta_now - dtheta_end(k);
            ct = cos(theta_now);
            st = sin(theta_now);
            stheta[k] = st;
            ctheta[k] = ct;
            Eigen::Matrix2d car_R, car_R_dot;
            car_R << ct, -st, st, ct;
            car_R_dot << -st, -ct, ct, -st;
            h = trailer->length[k+1] / 2.0;
            bps.clear();
            if (k % 2 == 1)
            {
                bps.push_back(Eigen::Vector2d(trailer->length[k+1] - trailer->rear_length, w));
                bps.push_back(Eigen::Vector2d(trailer->length[k+1] - trailer->rear_length, -w));
                bps.push_back(Eigen::Vector2d(-trailer->rear_length, -w));
                bps.push_back(Eigen::Vector2d(-trailer->rear_length, w));
            }
            else
            {
                bps.push_back(Eigen::Vector2d(0.1, w));
                bps.push_back(Eigen::Vector2d(0.1, -w));
                bps.push_back(Eigen::Vector2d(-0.1, -w));
                bps.push_back(Eigen::Vector2d(-0.1, w));
            }
            
            traileri(0) = traileri(0) - trailer->Lhead[k]*ct - trailer->Ltail[k]*ct_pre;
            traileri(1) = traileri(1) - trailer->Lhead[k]*st - trailer->Ltail[k]*st_pre;
            for (int j=0; j<4; j++)
            {
                for (int i=0; i<4; i++)
                {
                    double tail_cost = 0.0;
                    double tail_mu = mu[non_equal_idx];
                    bp = traileri + car_R * bps[j];
                    normal = trailer->terminal_area.col(i).head(2);
                    gx[non_equal_idx] = normal.dot(bp - trailer->terminal_area.col(i).tail(2)) * scale_cx(constrain_idx);
                    if (rho * gx[non_equal_idx] + tail_mu > 0)
                    {
                        tail_cost = getAugmentedCost(gx[non_equal_idx], tail_mu) * inner_weight_tail;
                        double aug_grad = getAugmentedGrad(gx[non_equal_idx], tail_mu) * scale_cx(constrain_idx)* inner_weight_tail;
                        grad_r2_state[k] += aug_grad * normal;
                        gdTail(k) += aug_grad * normal.dot(car_R_dot*bps[j]);
                    }
                    else
                    {
                        tail_cost = -0.5 * tail_mu * tail_mu / rho * inner_weight_tail;
                    }
                    cost += tail_cost;
                    non_equal_idx++;
                    constrain_idx++;
                }
            }
        }

        for (int k=TRAILER_NUM-1; k>=0; k--)
        {
            if (k>0)
            {
                grad_r2_state[k-1] += grad_r2_state[k];
                gdTail(k-1) += trailer->Ltail[k] * grad_r2_state[k].dot(Eigen::Vector2d(stheta[k-1], -ctheta[k-1]));
            }
            gdTail(k) += trailer->Lhead[k] * grad_r2_state[k].dot(Eigen::Vector2d(stheta[k], -ctheta[k]));
        }

        gradTailtractor_temp.head(2) += grad_r2_state[0];
        gradTailtractor_temp[2] += trailer->Ltail[0] * grad_r2_state[0].dot(Eigen::Vector2d(sin(Tailtractor_temp(2)), -cos(Tailtractor_temp(2))));

        return;
    }

    int ArcOpt::earlyExit(void* ptrObj, const Eigen::VectorXd& x, const Eigen::VectorXd& grad, 
                                const double fx, const double step, int k, int ls)
    {
        ArcOpt& obj = *(ArcOpt*)ptrObj;

        if (obj.in_debug)
        {
            const double& tau = x(0);
            int opt_var_idx = obj.dim_time;

            Eigen::Map<const Eigen::MatrixXd> Ppos(x.data() + opt_var_idx, 2, obj.piece_num_pos - 1);
            opt_var_idx += 2*(obj.piece_num_pos-1);

            Eigen::Map<const Eigen::MatrixXd> Ptheta(x.data()+opt_var_idx, TRAILER_NUM, obj.piece_num_theta-1);
            opt_var_idx += TRAILER_NUM * (obj.piece_num_theta-1);

            Eigen::Map<const Eigen::VectorXd> Varc(x.data()+opt_var_idx, obj.piece_num_pos);
            opt_var_idx += obj.piece_num_pos;

            Eigen::Map<const Eigen::VectorXd> Vphi(x.data()+opt_var_idx, TRAILER_NUM);
            opt_var_idx += TRAILER_NUM;
            
            Eigen::Map<const Eigen::VectorXd> Tailtractor(x.data()+opt_var_idx, 3);
            opt_var_idx += 3;
            
            // get T from τ, generate MINCO trajectory
            Eigen::VectorXd Tarc, Ttheta, dtheta_end, Arc;
            Tarc.resize(obj.piece_num_pos);
            Arc.resize(obj.piece_num_pos);
            obj.calTfromTauUni(tau, Tarc);
            obj.calTfromTau(Varc, Arc);
            Ttheta.resize(obj.piece_num_theta);
            obj.calTfromTauUni(tau, Ttheta);
            dtheta_end.resize(TRAILER_NUM);
            Eigen::MatrixXd init_arc, end_arc;
            init_arc.resize(1, 3);
            end_arc.resize(1, 3);
            init_arc << 0.0, obj.start_v, 0.0;
            end_arc << Arc.sum(), 0.0, 0.0;
            Eigen::MatrixXd Parc;
            Parc.resize(1, obj.piece_num_pos-1);
            for (int i=0; i<obj.piece_num_pos-1; i++)
            {
                if (i==0)
                    Parc(0, i) = Arc(i);
                else
                    Parc(0, i) = Arc(i) + Parc(i-1);
            }
            obj.arc_opt.generate(init_arc, end_arc, Parc, Tarc);

            // std::cout<<"time = "<<Tarc.transpose()<<std::endl;
            // std::cout<<"arc = "<<Arc.transpose()<<std::endl;
            // std::cout<<"parc = "<<Parc<<std::endl;

            Eigen::MatrixXd trailer_end;    trailer_end.resize(TRAILER_NUM, 3);    trailer_end.setZero();
            double theta_last = Tailtractor[2];
            for (size_t i=0; i<TRAILER_NUM; i++)
            {
                dtheta_end(i) = obj.sigmoidC2(Vphi(i));
                trailer_end.col(0)[i] = theta_last - dtheta_end(i);
                while (obj.end_theta.col(0)[i] - trailer_end.col(0)[i] >= M_PI_2)
                {
                    trailer_end.col(0)[i] += M_PI * 2.0;
                }
                while (obj.end_theta.col(0)[i] - trailer_end.col(0)[i] <= -M_PI_2)
                {
                    trailer_end.col(0)[i] -= M_PI * 2.0;
                }
                theta_last = trailer_end.col(0)[i];
            }

            Eigen::MatrixXd tractor_end = obj.end_pos;
            tractor_end.col(0) = Tailtractor.head(2);
            tractor_end.col(1)[0] = cos(Tailtractor[2]);
            tractor_end.col(1)[1] = sin(Tailtractor[2]);
            obj.head_opt.generate(obj.init_pos, tractor_end, Ppos, Arc);
            obj.tails_opt.generate(obj.init_theta, trailer_end, Ptheta, Ttheta);

            // ArcTraj temp_traj = obj.getTraj();
            // obj.pubDebugTraj(temp_traj);

            // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return k > obj.inner_max_iter;
    }
}