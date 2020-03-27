//Include system libraries
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sstream>

//Include user msgs
#include <gtddp_drone_msgs/Status.h>

//Include package libraries
#include "gtddp_drone/gtddp_lib/Constants.h"
#include "gtddp_drone/pursuit_control.h"

#define MAX_BUFFER 100

int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "pursuit_control_node");

    //Initialize this node
    ros::NodeHandle control_node;

    // Update Constants from launch file
    ConstantLoader loader(control_node);

    const int SIMULATION = control_node.param("pursuit_control_node/is_simulation", 0);
    const int OPEN_LOOP = control_node.param("pursuit_control_node/is_open_loop", 0);

    //Advertise control output, status, and landing mode
    ros::Publisher ctrl_sig_pub = control_node.advertise<geometry_msgs::Twist>(control_node.resolveName("/cmd_vel"), MAX_BUFFER);
    ros::Publisher stat_pub = control_node.advertise<gtddp_drone_msgs::Status>(control_node.resolveName("/gtddp_drone/status"), MAX_BUFFER);
    ros::Publisher land_pub = control_node.advertise<std_msgs::Empty>(control_node.resolveName("/ardrone/land"), MAX_BUFFER);

    //Set up the control calculator
    PursuitControl control_calc(ctrl_sig_pub, stat_pub, SIMULATION);

    //Set up callbacks for the current trajectory topic and the state estimation
    ros::Subscriber traj_sub = control_node.subscribe(control_node.resolveName("/gtddp_drone/trajectory"), MAX_BUFFER, &PursuitControl::trajectory_callback, &control_calc);
    ros::Subscriber estimate_sub;

    ros::Timer update_timer;

    //If this is a simulation, subscribe to the exact state update
    if(SIMULATION)
    {
        ROS_INFO("Simulation mode active");
        estimate_sub = control_node.subscribe(control_node.resolveName("/ground_truth/state"), MAX_BUFFER, &PursuitControl::state_estimate_callback, &control_calc);
    }
    else
    {
        estimate_sub = control_node.subscribe(control_node.resolveName("/vicon/ardrone1/odom"), MAX_BUFFER, &PursuitControl::state_estimate_callback, &control_calc);
    }

    //Set up a timer to call the control calculation function at the appropriate update rate
    if(OPEN_LOOP)
    {
        update_timer = control_node.createTimer(ros::Duration(0.001), &PursuitControl::open_loop_control, &control_calc, false);
    }
    else
    {
        update_timer = control_node.createTimer(ros::Duration(0.001), &PursuitControl::recalculate_control_callback, &control_calc, false);
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
