//Include system libraries
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sstream>

//Include package libraries
#include "gtddp_drone/gtddp_lib/Constants.h"
#include "gtddp_drone/control_calc.h"

#define MAX_BUFFER 100
//#define SIMULATION 1

int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "gtddp_control_node");

    //Initialize this node
    ros::NodeHandle control_node;

    // Update Constants from launch file
    ConstantLoader loader(control_node);

    const int SIMULATION = control_node.param("/gtddp_control_node/is_simulation", 0);
    const int OPEN_LOOP = control_node.param("gtddp_control_node/is_open_loop", 0);

    //Advertise control output, status, and landing mode
    ros::Publisher ctrl_sig_pub = control_node.advertise<geometry_msgs::Twist>(control_node.resolveName("/cmd_vel"), 1);

    //Set up the control calculator
    ControlCalculator control_calc(ctrl_sig_pub, SIMULATION);

    //Set up callbacks for the current trajectory topic and the state estimation
    ros::Subscriber traj_sub = control_node.subscribe(control_node.resolveName("/gtddp_drone/trajectory"), 1, &ControlCalculator::trajectory_callback, &control_calc);
    ros::Subscriber estimate_sub;

    ros::Timer update_timer;

    //If this is a simulation, subscribe to the exact state update
    if(SIMULATION)
    {
        ROS_INFO("Simulation mode active");
        estimate_sub = control_node.subscribe(control_node.resolveName("/ground_truth/state"), 1, &ControlCalculator::state_estimate_callback, &control_calc);
    }
    else
    {
        estimate_sub = control_node.subscribe(control_node.resolveName("/vicon/ardrone1/odom"), 1, &ControlCalculator::state_estimate_callback, &control_calc);
    }

    //Set up a timer to call the control calculation function at the appropriate update rate
    if(OPEN_LOOP)
    {
        update_timer = control_node.createTimer(ros::Duration(0.001), &ControlCalculator::open_loop_control, &control_calc, false);
    }
    else
    {
        update_timer = control_node.createTimer(ros::Duration(0.001), &ControlCalculator::recalculate_control_callback, &control_calc, false);
    }

    control_calc.set_timer(update_timer);

    //Create an empty message
    std_msgs::Empty empty_msg;

    //Launch the drone with a takeoff command
    ros::Publisher takeoff_pub = control_node.advertise<std_msgs::Empty>("/ardrone/takeoff", MAX_BUFFER);

    //Wait for there to be a subscriber before publishing the takeoff command
    while(takeoff_pub.getNumSubscribers() == 0)
    {
        sleep(1);
    }

    //Once we have locked to the ardrone, send the takeoff command
    takeoff_pub.publish(empty_msg);

    //Pump multithreaded callbacks
    ros::MultiThreadedSpinner spinner;
    spinner.spin();

    //Pump callbacks (single thread)
    //ros::spin();

    return 0;
}
