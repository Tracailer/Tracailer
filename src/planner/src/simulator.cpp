#include <iostream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <visualization_msgs/Marker.h>
#include "planner/trailer.hpp"
#include "planner/TrailerState.h"
#include "tf/transform_broadcaster.h"

// ros interface
tf::TransformBroadcaster* broadcaster;
ros::Subscriber command_sub, set_sub;
ros::Publisher  odom_pub, sensor_odom_pub;
ros::Timer simulate_timer;
ros::Time get_cmdtime;

// simulator variables
std::default_random_engine generator;
std::normal_distribution<double> distribution{0.0, 1.0};
ackermann_msgs::AckermannDrive im_cmd;
vector<ackermann_msgs::AckermannDrive> cmd_buff;
trailer_planner::Trailer trailer;
Eigen::VectorXd state;
bool rcv_cmd = false;

// simulator parameters
double time_resolution = 0.01;
double time_delay = 0.0;
double noise_std = 0.0;
double max_speed = 2.0;
double max_steer = 0.7;

void normYaw(double& th)
{
	while (th > M_PI)
		th -= M_PI * 2;
	while (th < -M_PI)
		th += M_PI * 2;
}

double guassRandom(double std)
{
	return std * distribution(generator);
}

void rcvCmdCallBack(const ackermann_msgs::AckermannDriveConstPtr msg)
{	
	if (rcv_cmd==false)
	{
		rcv_cmd = true;
		cmd_buff.push_back(*msg);
		get_cmdtime = ros::Time::now();
	}
	else
	{
		cmd_buff.push_back(*msg);
		if ((ros::Time::now() - get_cmdtime).toSec() > time_delay)
		{
			im_cmd = cmd_buff[0];
			cmd_buff.erase(cmd_buff.begin());
		}
	}
}

void rcvSetCallBack(const planner::TrailerStateConstPtr msg)
{	
	state[0] = msg->odoms[0].pose.pose.position.x;
	state[1] = msg->odoms[0].pose.pose.position.y;
	for (int i=0; i<TRAILER_NUM+1; i++)
		state[2+i] = msg->odoms[i].pose.pose.position.z;
	return;
}

void simCallback(const ros::TimerEvent &e)
{
    planner::TrailerState new_odom;
	nav_msgs::Odometry odom_temp;
	odom_temp.header.frame_id = "world";

	double v = max(min((double)im_cmd.speed, max_speed), -max_speed) + guassRandom(noise_std);
	double delta = max(min((double)im_cmd.steering_angle, max_steer), -max_steer) + guassRandom(noise_std);
	Eigen::VectorXd new_state, new_se2_state, v_state, w_state;
	new_state = state;
	v_state.resize(TRAILER_NUM+1);
	w_state.resize(TRAILER_NUM+1);

	// state transition use vel
	double w = v * tan(delta) / trailer.wheel_base;
	double y = time_resolution * w;

	if (fabs(w) > 1e-4)
	{
		new_state(0) = state(0) + v / w * (sin(state(2)+y) - sin(state(2)));
		new_state(1) = state(1) - v / w * (cos(state(2)+y) - cos(state(2)));
		new_state(2) = state(2) + y;
		normYaw(new_state(2));
	}
	else
	{
		new_state(0) = state(0) + v * time_resolution * cos(state(2));
		new_state(1) = state(1) + v * time_resolution * sin(state(2));
		new_state(2) = state(2);
	}
	v_state[0] = v;
	w_state[0] = w;

	for (size_t i=0; i<TRAILER_NUM; i++)
	{
		double sthetad = sin(state(i+2)-state(i+3));
		double cthetad = cos(state(i+2)-state(i+3));
		double w_temp = w;
		w = (v * sthetad - trailer.Ltail[i] * w * cthetad) /trailer.Lhead[i];
		v = v * cthetad + trailer.Ltail[i] * w_temp * sthetad;
		v_state[i+1] = v;
		w_state[i+1] = w;
		new_state(i+3) = state(i+3) + w * time_resolution;
		normYaw(new_state(i+3));
	}

	trailer.gainSE2State(new_state, new_se2_state);
	for (size_t i=0; i<TRAILER_NUM+1; i++)
	{
		odom_temp.pose.pose.position.x = new_se2_state[3*i];
		odom_temp.pose.pose.position.y = new_se2_state[3*i+1];
		double yy = new_se2_state[3*i+2];
		odom_temp.pose.pose.orientation.w = cos(yy/2.0);
		odom_temp.pose.pose.orientation.x = 0.0;
		odom_temp.pose.pose.orientation.y = 0.0;
		odom_temp.pose.pose.orientation.z = sin(yy/2.0);
		odom_temp.twist.twist.angular.z = w_state[i];
		odom_temp.twist.twist.linear.x = v_state[i];
		new_odom.odoms.push_back(odom_temp);
	}
	trailer.showTrailer(new_state, 1);
	new_odom.time_now = ros::Time::now();
	odom_pub.publish(new_odom);
	state = new_state;
	sensor_odom_pub.publish(new_odom.odoms[0]);

	tf::Transform transform;
	transform.setOrigin(tf::Vector3(new_odom.odoms[0].pose.pose.position.x, 
									new_odom.odoms[0].pose.pose.position.y, 	
									new_odom.odoms[0].pose.pose.position.z));
	transform.setRotation(tf::Quaternion(new_odom.odoms[0].pose.pose.orientation.x,
										new_odom.odoms[0].pose.pose.orientation.y,
										new_odom.odoms[0].pose.pose.orientation.z,
										new_odom.odoms[0].pose.pose.orientation.w));
	ros::Time now = ros::Time::now();
	broadcaster->sendTransform(tf::StampedTransform(transform, now, string("world"), "robot"));
	return;
} 

// main loop
int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "simulator_node");
    ros::NodeHandle nh("~");

    trailer.init(nh);

    state.resize(TRAILER_NUM+3);
	for (size_t i=0; i<TRAILER_NUM+3; i++)
		state[i] = trailer.pthetas[i];

	im_cmd.speed = 0.0;
	im_cmd.steering_angle = 0.0;

	trailer.showTrailer(state, 1);

    command_sub  = nh.subscribe("cmd", 1000, rcvCmdCallBack);
	set_sub = nh.subscribe("/trailer_set", 1000, rcvSetCallBack);
	odom_pub = nh.advertise<planner::TrailerState>("odom", 10);
	sensor_odom_pub = nh.advertise<nav_msgs::Odometry>("sensor_odom", 10);
    simulate_timer = nh.createTimer(ros::Duration(time_resolution), simCallback);
	tf::TransformBroadcaster b;
	broadcaster = &b;

	ros::spin();

    return 0;

}