//System headers
#include "ros/ros.h"

//Package defined headers
#include "gtddp_drone/gtddp_lib/Constants.h"
#include "gtddp_drone_msgs/Trajectory.h"
#include "gtddp_drone_msgs/target.h"
#include "gtddp_drone/traj_optimizer.h"

//Define program constants
#define MAX_BUFFER 10
//#define SIMULATION 1


//ROS Variables
//Publishers
ros::Publisher cur_state_pub;
ros::Publisher init_pub;
ros::Publisher traj_pub;

//Services
ros::ServiceClient target_client;

//Subscribers
ros::Subscriber control_status_sub;
ros::Subscriber estimate_sub;
ros::Subscriber go_button;

//Timers
ros::Timer update_timer;

/**
 *
 */
int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "gtddp_optimize_node");

    //Initialize this node
    ros::NodeHandle traj_node;

    // Update Constants from launch file
    ConstantLoader loader(traj_node);

    const int SIMULATION    = traj_node.param("/gtddp_optimize_node/is_simulation", 0);
    const int GENERATE_TRAJ = traj_node.param("/gtddp_optimize_node/is_gen", 0);
    const int REAL_TIME     = traj_node.param("/gtddp_optimize_node/is_real_time", 0);
    const int NUM_GEN_LEGS  = traj_node.param("/gtddp_optimize_node/gen_legs", 0);
    const int OPEN_LOOP     = traj_node.param("/gtddp_optimize_node/is_open_loop", 0);

    //Advertise trajectory data
    traj_pub = traj_node.advertise<gtddp_drone_msgs::Trajectory>(traj_node.resolveName("/gtddp_drone/trajectory"), MAX_BUFFER);
    cur_state_pub = traj_node.advertise<gtddp_drone_msgs::state_data>(traj_node.resolveName("/gtddp_drone/cur_state_sim"), MAX_BUFFER);
    init_pub = traj_node.advertise<gtddp_drone_msgs::state_data>(traj_node.resolveName("/gtddp_drone/initial_conditions"), MAX_BUFFER);

    //Set up a service client for getting the next target point
    target_client = traj_node.serviceClient<gtddp_drone_msgs::target>(traj_node.resolveName("/gtddp_drone_target_trajectory/target_state"));

    //Wait for there to be a subscriber to init before optimizing
    while(init_pub.getNumSubscribers() == 0 && GENERATE_TRAJ)
    {
        sleep(1);
    }

    //Set up the trajectory optimizer
    Optimizer traj_optimizer(traj_pub, cur_state_pub, init_pub, target_client, GENERATE_TRAJ, REAL_TIME, OPEN_LOOP);
    traj_optimizer.set_num_legs(NUM_GEN_LEGS);

    //If a test is occuring, set up for a flight
    if(!GENERATE_TRAJ)
    {
        //Subscribe to the keyboard's GO button
        go_button = traj_node.subscribe(traj_node.resolveName("/gtddp_drone/start"), MAX_BUFFER, &Optimizer::init_optimizer, &traj_optimizer);

        //Subscribe to state estimation and target state topics
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

        //Subscribe to the control status (for iterative DDP use)
        control_status_sub = traj_node.subscribe(traj_node.resolveName("/gtddp_drone/status"), MAX_BUFFER, &Optimizer::status_callback, &traj_optimizer);

        //Set up timer for recalculations
        //If the optimization is happening in real time, run the optimization callbacks
        if(REAL_TIME)
        {
            update_timer = traj_node.createTimer(ros::Duration(1.0), &Optimizer::traj_update_callback, &traj_optimizer, false);
        }
        // If open loop, only read past control commands
        else if (OPEN_LOOP)
        {
            //Send the new trajectories at a faster update rate than the DDP would normally run
            update_timer = traj_node.createTimer(ros::Duration(0.3), &Optimizer::open_loop_traj_callback, &traj_optimizer, false);
        }
        //Otherwise read full state, control commands, and gains from the flight file
        else
        {
            //Send the new trajectories at a faster update rate than the DDP would normally run
            update_timer = traj_node.createTimer(ros::Duration(0.3), &Optimizer::offline_traj_callback, &traj_optimizer, false);
        }
    }
    //If the trajectory is being generated for an offline run, optimize small trajectories but only output to a file
    else
    {
        //Subscribe to nothing because this is a generated trajectory
        //Optimize the trajectory step-by-step using a timer
        update_timer = traj_node.createTimer(ros::Duration(1.0), &Optimizer::traj_update_callback, &traj_optimizer, false);
    }

    //Pump callbacks
    ros::MultiThreadedSpinner spinner;
    spinner.spin();

    //ros::spin();

    return 0;
}
