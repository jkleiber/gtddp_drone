//System headers
#include "ros/ros.h"
#include "std_msgs/String.h"

//Package defined headers
#include "traj_optimizer.h"

//Define program constants
#define MAX_BUFFER 100

/**
 * 
 */
int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "gtddp_traj_optimize");

    //Initialize this node
    ros::NodeHandle traj_node;

    //Advertise trajectory data
    //TODO: Change message type
    ros::Publisher traj_pub = traj_node.advertise<std_msgs::String>("trajectory", MAX_BUFFER);

    //Set up the trajectory optimizer
    Optimizer traj_optimizer(traj_pub);

    //Subscribe to state estimation and target state topics
    ros::Subscriber estimate_sub = traj_node.subscribe("state_estimate", MAX_BUFFER, &Optimizer::state_estimate_callback, &traj_optimizer);
    ros::Subscriber target_sub = traj_node.subscribe("target_state", MAX_BUFFER, &Optimizer::target_state_callback, &traj_optimizer);

    //Set up timer for recalculations
    ros::Timer update_timer = traj_node.createTimer(ros::Duration(0.05), &Optimizer::traj_update_callback, &traj_optimizer, false);

    //Pump callbacks
    ros::spin();

    return 0;
}