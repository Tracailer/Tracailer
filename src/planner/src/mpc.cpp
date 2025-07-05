#include "planner/mpc.h"

using namespace std;
 
void MPC::init(ros::NodeHandle &nh)
{
    vector<double> Q_ ;
    vector<double> R_;
    vector<double> Rd_;
    double cpt_time; 

    trailer.init(nh);
    nh.param("mpc/cpt_time", cpt_time, -1.0);
    nh.param("mpc/dt", dt, -1.0);
    nh.param("mpc/predict_steps", Npre, -1);
    nh.param("mpc/max_speed", max_speed, -1.0);
    nh.param("mpc/min_speed", min_speed, -1.0);
    nh.param("mpc/max_accel", max_accel, -1.0);
    nh.param("mpc/max_dsteer", max_dsteer, -1.0);
    nh.param("mpc/delay_num", delay_num, -1);
    nh.param<std::vector<double>>("mpc/matrix_q", Q_, std::vector<double>());
    nh.param<std::vector<double>>("mpc/matrix_r", R_, std::vector<double>());
    nh.param<std::vector<double>>("mpc/matrix_rd", Rd_, std::vector<double>());

    DM q({Q_[0], Q_[1], Q_[2], Q_[3]});
    for (size_t i=0; i<TRAILER_NUM-1; i++)
        q = vertcat(q, q(3));
    Q = diag(q);
    R = diag(DM({R_[0], R_[1]}));
    Rd = diag(DM({Rd_[0], Rd_[1]}));
    U_min = DM({min_speed, -trailer.max_steer});
    U_max = DM({max_speed, trailer.max_steer});
    dU_min = DM({-max_accel, -max_dsteer});
    dU_max = DM({max_accel, max_dsteer});
    x_0.resize(3+TRAILER_NUM, 1);

    u_0.resize(2, 1);
    u_0 = {0, 0};
    X_sol = repmat(x_0, 1, Npre);
    U_sol = repmat(u_0, 1, Npre);

    has_odom = false;
    receive_traj = false;
    for (int i=0; i<delay_num; i++)
        output_buff.push_back(Eigen::Vector2d::Zero());
    cmd.speed = 0.0;
    cmd.steering_angle = 0.0;

    cmd_pub = nh.advertise<ackermann_msgs::AckermannDrive>("cmd", 200);
    predict_pub = nh.advertise<visualization_msgs::Marker>("predict_path", 10);
    ref_pub = nh.advertise<visualization_msgs::Marker>("reference_path", 10);

    cmd_timer = nh.createTimer(ros::Duration(cpt_time), &MPC::cmdCallback, this);

    odom_sub = nh.subscribe("odom", 100, &MPC::rcvOdomCallBack, this);
        arc_traj_sub = nh.subscribe("arc_traj", 100, &MPC::rcvArcTrajCallBack, this);
    return;
}

void MPC::rcvArcTrajCallBack(planner::ArcTrailerTrajConstPtr msg)
    {
        traj_analyzer.setTraj(msg);
        receive_traj = true;
        
    return;
}

void MPC::rcvOdomCallBack(planner::TrailerStatePtr msg)
{
    has_odom = true;
    Eigen::VectorXd s;
    s.resize(TRAILER_NUM+3);
    s[0] = msg->odoms[0].pose.pose.position.x;
    s[1] = msg->odoms[0].pose.pose.position.y;
    s[2] = 2.0 * atan2(msg->odoms[0].pose.pose.orientation.z, msg->odoms[0].pose.pose.orientation.w);
    for (int i=1; i<=TRAILER_NUM; i++)
        s[2+i] = 2.0 * atan2(msg->odoms[i].pose.pose.orientation.z, msg->odoms[i].pose.pose.orientation.w);
    now_state.data = s;
    se2_now_state.resize(3*(TRAILER_NUM+1));
    trailer.gainSE2State(now_state.data, se2_now_state);
    return;
}

void MPC::cmdCallback(const ros::TimerEvent &e)
{
    if (!has_odom || !receive_traj)
        return;
    
        xref = traj_analyzer.getRefPoints(Npre, dt);
        if (traj_analyzer.at_goal || xref.empty())
        {
            cmd.speed = 0.0;
            cmd.steering_angle = 0.0;
            for (size_t i=0; i<TRAILER_NUM+3; i++)
                x_0(i) = now_state.data(i);
            u_0 = {0, 0};
            X_sol = repmat(x_0, 1, Npre);
            U_sol = repmat(u_0, 1, Npre);
        }
        else
        {
            smooth_yaw(xref);
            getCmd();
    }

    cmd_pub.publish(cmd);
}

void MPC::getCmd(void)
{
    nlp = casadi::Opti();
    casadi::Dict options;
    casadi::Dict qp_options;
    // options["ipopt.print_level"] = 0;
    options["print_status"] = false;
    options["print_time"] = false;
    options["print_header"] = false;
    options["print_iteration"] = false;
    options["verbose"] = false;
    options["verbose_init"] = false;
    qp_options["printLevel"] = "none";
    qp_options["sparse"] = true;
    qp_options["error_on_fail"] = false;
    options["qpsol_options"] = qp_options;
        
    X = nlp.variable(TRAILER_NUM+3, Npre);
    U = nlp.variable(2, Npre);
    J = 0;
    MX X_0 = nlp.parameter(TRAILER_NUM+3, 1);
    MX x_next;
    Slice all;

    for (int i = 0; i < Npre; i++)
    {
        MX ref_x = xref[i].x();
        MX ref_y = xref[i].y();
        MX ref_v = 0.0;
        MX ref_steer = 0.0;
        MX ref_theta = xref[i].theta0();
        MX ref_state = vertcat(ref_x, ref_y, ref_theta);
        for (size_t j=0; j<TRAILER_NUM; j++)
        {
            MX th = xref[i].data(3+j);
            ref_state = vertcat(ref_state, th);
        }
        MX ref_u = vertcat(ref_v, ref_steer);

        if (i==0)
            x_next = stateTrans(X_0, U(all, i));
        else
            x_next = stateTrans(X(all, i-1), U(all, i));
        nlp.subject_to(X(all, i) == x_next);
        nlp.subject_to(U_min <= U(all, i) <= U_max);
        
        MX delta_x = ref_state - X(all, i);
        MX delta_u = ref_u - U(all, i);
        
        MX du;
        if (i > 0)
            du = U(all, i) - U(all, i - 1);
        else
            du = U(all, i) - u_0;
        nlp.subject_to(dU_min <= du <= dU_max);

        J = J + mtimes(delta_x.T(), mtimes(Q, delta_x));
        J = J + mtimes(delta_u.T(), mtimes(R, delta_u));
        J = J + mtimes(du.T(), mtimes(Rd, du));
    }

    for (size_t i=0; i<TRAILER_NUM+3; i++)
        x_0(i) = now_state.data(i);
    nlp.set_value(X_0, x_0);
    nlp.set_initial(X, X_sol);
    nlp.set_initial(U, U_sol);
    nlp.solver("sqpmethod", options);
    // nlp.solver("ipopt", options);
    nlp.minimize(J);

    try
    {
        ros::Time a1 = ros::Time::now();
        const casadi::OptiSol sol = nlp.solve();
        double sol_time = (ros::Time::now() - a1).toSec() *1000.0;
        if (sol_time > cpt_time * 1000.0)
        {
            ROS_WARN("MPC solve time: %f ms", sol_time);
        }
        X_sol = sol.value(X);
        U_sol = sol.value(U);
        DM cmd_0 = U_sol(all, 0);
        u_0 = cmd_0;
        cmd.speed = (double)cmd_0(0, 0);
        cmd.steering_angle = (double)cmd_0(1, 0);
    }
    catch(const std::exception& e)
    {
        // ROS_WARN("solver error, but we ignore.");
        ;
    }
    
    drawRefPath();
    drawPredictPath();

    if (delay_num>0)
    {
        output_buff.erase(output_buff.begin());
        output_buff.push_back(Eigen::Vector2d(cmd.speed, cmd.steering_angle));
    }
}

int main( int argc, char * argv[] )
{ 
  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle nh("~");

  MPC mpc_tracker;

  mpc_tracker.init(nh);

  ros::spin();

  return 0;
}