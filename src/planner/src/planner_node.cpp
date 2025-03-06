#include "planner/planner.h"
#include <ros/ros.h>

using namespace trailer_planner;

int main( int argc, char * argv[] )
{ 
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh("~");

    Planner planner;
    
    planner.init(nh);

    // ros::MultiThreadedSpinner spinner(2);
    // spinner.spin();

    ros::spin();

    return 0;
}