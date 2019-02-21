//Include system libraries
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <sstream>

//Include package libraries
#include "gtddp_drone/control_calc.h"

#define MAX_BUFFER 100
#define SIMULATION 1

int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "gtddp_control_node");

    //Initialize this node
    ros::NodeHandle control_node;

    //Advertise control output and landing mode
    ros::Publisher ctrl_sig_pub = control_node.advertise<geometry_msgs::Twist>(control_node.resolveName("/cmd_vel"), MAX_BUFFER);
    ros::Publisher land_pub = control_node.advertise<std_msgs::Empty>(control_node.resolveName("/ardrone/land"), MAX_BUFFER);

    //Set up the control calculator
    ControlCalculator control_calc(ctrl_sig_pub, land_pub);

    //Set up callbacks for the current trajectory topic and the state estimation
    ros::Subscriber traj_sub = control_node.subscribe(control_node.resolveName("/gtddp_drone/trajectory"), MAX_BUFFER, &ControlCalculator::trajectory_callback, &control_calc);
    ros::Subscriber estimate_sub;

    //If this is a simulation, subscribe to the exact state update
    if(SIMULATION)
    {
        ROS_INFO("Simulation mode active");
        estimate_sub = control_node.subscribe(control_node.resolveName("/ground_truth/state"), MAX_BUFFER, &ControlCalculator::ground_truth_callback, &control_calc);
    }
    else
    {
        estimate_sub = control_node.subscribe(control_node.resolveName("/ardrone/predictedPose"), MAX_BUFFER, &ControlCalculator::state_estimate_callback, &control_calc);
    }
    
    //Set up a timer to call the control calculation function at the appropriate update rate
    ros::Timer update_timer = control_node.createTimer(ros::Duration(0.01), &ControlCalculator::recalculate_control_callback, &control_calc, false);

    //Set up and change the settings of tum_ardrone to use the Vicon system
    ros::Publisher tum_settings = control_node.advertise<std_msgs::String>(control_node.resolveName("switch_control"), 10);
    std_msgs::String str;
    std::stringstream ss("Vicon");
    str.data = ss.str();
    tum_settings.publish(str);

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

    //Pump callbacks
    ros::spin();

    return 0;
}