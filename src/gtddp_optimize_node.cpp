//System headers
#include "ros/ros.h"

#include <std_msgs/Bool.h>

//Package defined headers
#include "gtddp_drone/gtddp_lib/Constants.h"
#include "gtddp_drone_msgs/Trajectory.h"
#include "gtddp_drone_msgs/target.h"
#include "gtddp_drone/traj_optimizer.h"

//Define program constants
#define MAX_BUFFER 10


//ROS Variables
//Publishers
ros::Publisher cur_state_pub;
ros::Publisher init_pub;
ros::Publisher traj_pub;
ros::Publisher file_pub;

//Services
ros::ServiceClient target_client;

//Subscribers
ros::Subscriber estimate_sub, estimate_sub_2;
ros::Subscriber go_button;

//Timers
ros::Timer update_timer;
ros::Timer file_timer;

Optimizer *traj_optimizer;


void fileCheckerCallback(const ros::TimerEvent& event)
{
    std_msgs::Bool bool_msg;

    // Have we run out of data to read?
    bool file_status = !traj_optimizer->is_offline_good();
    bool_msg.data = file_status;

    // Publish the file status
    file_pub.publish(bool_msg);
}


/**
 *
 */
int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "gtddp_optimize_node");

    //Initialize this node
    ros::NodeHandle traj_node;

    const int SIMULATION    = traj_node.param("/gtddp_optimize_node/is_simulation", 0);
    const int GENERATE_TRAJ = traj_node.param("/gtddp_optimize_node/is_gen", 0);
    const int REAL_TIME     = traj_node.param("/gtddp_optimize_node/is_real_time", 0);
    const int NUM_GEN_LEGS  = traj_node.param("/gtddp_optimize_node/gen_legs", 0);
    const int OPEN_LOOP     = traj_node.param("/gtddp_optimize_node/is_open_loop", 0);

    //Advertise trajectory data
    traj_pub = traj_node.advertise<gtddp_drone_msgs::Trajectory>(traj_node.resolveName("/gtddp_drone/trajectory"), MAX_BUFFER);
    cur_state_pub = traj_node.advertise<gtddp_drone_msgs::state_data>(traj_node.resolveName("/gtddp_drone/cur_state_sim"), MAX_BUFFER);
    init_pub = traj_node.advertise<gtddp_drone_msgs::state_data>(traj_node.resolveName("/gtddp_drone/initial_conditions"), MAX_BUFFER);
    file_pub = traj_node.advertise<std_msgs::Bool>(traj_node.resolveName("/gtddp/control/empty"), 1);

    //Set up a service client for getting the next target point
    target_client = traj_node.serviceClient<gtddp_drone_msgs::target>(traj_node.resolveName("/gtddp_drone_target_trajectory/target_state"));

    // Update Constants from launch file
    ConstantLoader loader(traj_node);

    // Only use target trajectory node in trajectory tracking applications
    bool use_target_traj = Constants::ddp_selector.compare("pursuit")
                        && Constants::ddp_selector.compare("ccddp_cart_pole")
                        && Constants::ddp_selector.compare("gtddp_cart_pole");

    // Wait for there to be a subscriber to init before optimizing
    // In pursuit-evasion, we do not need to wait for the init pub, so this will automatically continue in this case
    while(init_pub.getNumSubscribers() == 0 && GENERATE_TRAJ && use_target_traj)
    {
        sleep(1);
    }

    //Set up the trajectory optimizer
    traj_optimizer = new Optimizer(traj_pub, cur_state_pub, init_pub, target_client, GENERATE_TRAJ, REAL_TIME, OPEN_LOOP);
    traj_optimizer->set_num_legs(NUM_GEN_LEGS);

    //If a test is occuring, set up for a flight
    if(!GENERATE_TRAJ)
    {
        //Subscribe to the keyboard's GO button
        go_button = traj_node.subscribe(traj_node.resolveName("/gtddp_drone/start"), 1, &Optimizer::init_optimizer, traj_optimizer);

        //Subscribe to state estimation and target state topics
        //In simulations, subscribe to ground truth
        if(SIMULATION)
        {
            if(!Constants::ddp_selector.compare("pursuit"))
            {
                estimate_sub = traj_node.subscribe(traj_node.resolveName("/drone1/ground_truth/state"), 1, &Optimizer::state_estimate_callback, traj_optimizer);
                estimate_sub_2 = traj_node.subscribe(traj_node.resolveName("/drone2/ground_truth/state"), 1, &Optimizer::state_estimate_callback_2, traj_optimizer);
            }
            else
            {
                estimate_sub = traj_node.subscribe(traj_node.resolveName("ground_truth/state"), 1, &Optimizer::state_estimate_callback, traj_optimizer);
            }
        }
        //Otherwise subscribe to the vicon system
        else
        {
            estimate_sub = traj_node.subscribe(traj_node.resolveName("/drone1/vicon/odom"), 1, &Optimizer::state_estimate_callback, traj_optimizer);

            if(!Constants::ddp_selector.compare("pursuit"))
            {
                estimate_sub_2 = traj_node.subscribe(traj_node.resolveName("/drone2/vicon/odom"), 1, &Optimizer::state_estimate_callback_2, traj_optimizer);
            }
        }

        //Set up timer for recalculations
        //If the optimization is happening in real time, run the optimization callbacks
        if(REAL_TIME)
        {
            update_timer = traj_node.createTimer(ros::Duration(1.0), &Optimizer::traj_update_callback, traj_optimizer, false);
        }
        // If open loop, only read past control commands
        else if (OPEN_LOOP)
        {
            //Send the new trajectories at a faster update rate than the DDP would normally run
            update_timer = traj_node.createTimer(ros::Duration(0.3), &Optimizer::open_loop_traj_callback, traj_optimizer, false);
        }
        //Otherwise read full state, control commands, and gains from the flight file
        else
        {
            //Send the new trajectories at a faster update rate than the DDP would normally run
            update_timer = traj_node.createTimer(ros::Duration(0.3), &Optimizer::offline_traj_callback, traj_optimizer, false);
        }
    }
    //If the trajectory is being generated for an offline run, optimize small trajectories but only output to a file
    else
    {
        //Subscribe to nothing because this is a generated trajectory
        //Optimize the trajectory step-by-step using a timer
        if(!Constants::ddp_selector.compare("pursuit"))
        {
            std::cout << "Pursuit Mode Enabled\n";
            update_timer = traj_node.createTimer(ros::Duration(1.0), &Optimizer::pursuit_traj_callback, traj_optimizer, false);
        }
        else if(!Constants::ddp_selector.compare("ccddp_cart_pole") || !Constants::ddp_selector.compare("gtddp_cart_pole"))
        {
            std::cout << "Cart-Pole Mode Enabled\n";
            update_timer = traj_node.createTimer(ros::Duration(1.0), &Optimizer::cart_pole_traj_callback, traj_optimizer, false);
        }
        else
        {
            update_timer = traj_node.createTimer(ros::Duration(1.0), &Optimizer::traj_update_callback, traj_optimizer, false);
        }
    }


    // Set up the file timer watchdog
    file_timer = traj_node.createTimer(ros::Duration(0.5), &fileCheckerCallback, false);

    //Pump callbacks
    ros::MultiThreadedSpinner spinner;
    spinner.spin();

    //ros::spin();

    return 0;
}
