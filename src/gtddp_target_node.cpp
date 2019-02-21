//Include system libraries
#include <ros/ros.h>
#include <ros/console.h>
#include <sstream>
#include <vector>

//Include other libraries
#include <gtddp_drone/gtddp_lib/Constants.h>
#include <gtddp_drone/state_data.h>

#define MAX_BUFFER 100
#define SIMULATION 1

int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "gtddp_target_node");

    //Initialize this node
    ros::NodeHandle target_node;

    //Advertise control output and landing mode
    ros::Publisher target_pub = target_node.advertise<gtddp_drone::state_data>(target_node.resolveName("/gtddp_drone/target_state"), 10);
    
    //TODO: set up a timer for changing the target state

    //Establish a loop rate for the target node to run at
    ros::Rate loop_rate(100); //100 Hz

    //Create a state message
    gtddp_drone::state_data target_state;

    //Choose an initial target state
    //TODO: make this dynamic
    std::vector<double> state_vector(Constants::num_states, 0.0);
    state_vector[0] = 10;
    state_vector[1] = 10;
    state_vector[2] = 10;

    //Add the values to the initial state
    //TODO: make a cleaner way of doing this when paths are to be followed
    for(int i = 0; i < Constants::num_states; ++i)
        target_state.states[i] = state_vector[i];

    //ROS_INFO("New target state ready");
    
    //Wait for there to be a subscriber before publishing the takeoff command
    while(target_pub.getNumSubscribers() == 0)
    {
        sleep(1);
    }
    
    //Once we have locked to the ardrone, send the takeoff command
    target_pub.publish(target_state);
    //ROS_INFO("New target state published");

    //Run the target state loop
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}