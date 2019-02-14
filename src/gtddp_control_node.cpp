//Include system libraries
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

//Include package libraries
#include "gtddp_drone/control_calc.h"

#define MAX_BUFFER 100

int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "gtddp_control_node");

    //Initialize this node
    ros::NodeHandle control_node;

    //Advertise control output
    ros::Publisher ctrl_sig_pub = control_node.advertise<geometry_msgs::Twist>(control_node.resolveName("/ardrone/cmd_vel"), MAX_BUFFER);

    //Set up the control calculator
    ControlCalculator control_calc(ctrl_sig_pub);

    //Set up callbacks for the current trajectory topic and the state estimation
    ros::Subscriber traj_sub = control_node.subscribe(control_node.resolveName("/gtddp_drone/trajectory"), MAX_BUFFER, &ControlCalculator::trajectory_callback, &control_calc);
    ros::Subscriber estimate_sub = control_node.subscribe(control_node.resolveName("ardrone/predictedPose"), MAX_BUFFER, &ControlCalculator::state_estimate_callback, &control_calc);
    
    //Set up a timer to call the control calculation function at the appropriate update rate
    ros::Timer update_timer = control_node.createTimer(ros::Duration(0.01), &ControlCalculator::recalculate_control_callback, &control_calc, false);

    //Pump callbacks
    ros::spin();

    return 0;
}