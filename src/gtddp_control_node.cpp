//Include system libraries
#include "ros/ros.h"
#include "std_msgs/String.h"

//Include package libraries
#include "control_calc.h"

#define MAX_BUFFER 100

int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "gtddp_control_node");

    //Initialize this node
    ros::NodeHandle control_node;

    //Advertise control output
    ros::Publisher ctrl_sig_pub = control_node.advertise<std_msgs::String>("control_signal", MAX_BUFFER);

    //Set up the control calculator
    ControlCalculator control_calc(ctrl_sig_pub);

    //Set up callbacks for the current trajectory topic
    ros::Subscriber traj_sub = control_node.subscribe("trajectory", MAX_BUFFER, &ControlCalculator::trajectory_callback, &control_calc);

    //Pump callbacks
    ros::spin();

    return 0;
}