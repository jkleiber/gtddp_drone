//System headers
#include "ros/ros.h"

//Package defined headers
#include "gtddp_drone/Trajectory.h"
#include "gtddp_drone/traj_optimizer.h"

//Define program constants
#define MAX_BUFFER 100
#define SIMULATION 1

/**
 * 
 */
int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "gtddp_optimize_node");

    //Initialize this node
    ros::NodeHandle traj_node;

    //Advertise trajectory data
    ros::Publisher traj_pub = traj_node.advertise<gtddp_drone::Trajectory>(traj_node.resolveName("/gtddp_drone/trajectory"), MAX_BUFFER);

    //Set up the trajectory optimizer
    Optimizer traj_optimizer(traj_pub);

    //Subscribe to state estimation and target state topics
    ros::Subscriber estimate_sub;
    ros::Subscriber target_sub = traj_node.subscribe(traj_node.resolveName("gtddp_drone/target_state"), MAX_BUFFER, &Optimizer::target_state_callback, &traj_optimizer);

    //In simulations, subscribe to ground truth
    if(SIMULATION)
    {
        estimate_sub = traj_node.subscribe(traj_node.resolveName("ground_truth/state"), MAX_BUFFER, &Optimizer::ground_truth_callback, &traj_optimizer);
    }
    //Otherwise subscribe to the vicon system
    else
    {
        estimate_sub = traj_node.subscribe(traj_node.resolveName("ardrone/predictedPose"), MAX_BUFFER, &Optimizer::state_estimate_callback, &traj_optimizer);
    }

    //Set up timer for recalculations
    ros::Timer update_timer = traj_node.createTimer(ros::Duration(1.0), &Optimizer::traj_update_callback, &traj_optimizer, false);

    //Pump callbacks
    ros::spin();

    return 0;
}