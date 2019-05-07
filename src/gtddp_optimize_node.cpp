//System headers
#include "ros/ros.h"

//Package defined headers
#include "gtddp_drone_msgs/Trajectory.h"
#include "gtddp_drone_msgs/target.h"
#include "gtddp_drone/traj_optimizer.h"

//Define program constants
#define MAX_BUFFER 100
//#define SIMULATION 1

/**
 * 
 */
int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "gtddp_optimize_node");

    //Initialize this node
    ros::NodeHandle traj_node;

    const int SIMULATION = traj_node.param("/gtddp_optimize_node/is_simulation", 0);

    //Advertise trajectory data
    ros::Publisher traj_pub = traj_node.advertise<gtddp_drone_msgs::Trajectory>(traj_node.resolveName("/gtddp_drone/trajectory"), MAX_BUFFER);
    ros::Publisher cur_state_pub = traj_node.advertise<gtddp_drone_msgs::state_data>(traj_node.resolveName("/gtddp_drone/cur_state_sim"), MAX_BUFFER);
    ros::Publisher init_pub = traj_node.advertise<gtddp_drone_msgs::state_data>(traj_node.resolveName("/gtddp_drone/initial_conditions"), MAX_BUFFER);

    //Set up a service client for getting the next target point
    ros::ServiceClient target_client = traj_node.serviceClient<gtddp_drone_msgs::target>(traj_node.resolveName("/gtddp_drone_target_trajectory/target_state"));

    //Set up the trajectory optimizer
    Optimizer traj_optimizer(traj_pub, cur_state_pub, init_pub, target_client);

    //Subscribe to the keyboard's GO button
    ros::Subscriber go_button = traj_node.subscribe(traj_node.resolveName("/gtddp_drone/start"), MAX_BUFFER, &Optimizer::init_optimizer, &traj_optimizer);

    //Subscribe to state estimation and target state topics
    ros::Subscriber estimate_sub;
    ros::Subscriber control_status_sub = traj_node.subscribe(traj_node.resolveName("/gtddp_drone/status"), MAX_BUFFER, &Optimizer::status_callback, &traj_optimizer);
    //In simulations, subscribe to ground truth
    if(SIMULATION)
    {
        estimate_sub = traj_node.subscribe(traj_node.resolveName("ground_truth/state"), MAX_BUFFER, &Optimizer::state_estimate_callback, &traj_optimizer);
    }
    //Otherwise subscribe to the vicon system
    else
    {
        estimate_sub = traj_node.subscribe(traj_node.resolveName("/vicon/ardrone1/odom"), MAX_BUFFER, &Optimizer::state_estimate_callback, &traj_optimizer);
    }

    //Set up timer for recalculations
    ros::Timer update_timer = traj_node.createTimer(ros::Duration(1.0), &Optimizer::traj_update_callback, &traj_optimizer, false);

    //Pump callbacks
    ros::MultiThreadedSpinner spinner;
    spinner.spin();

    //ros::spin();

    return 0;
}