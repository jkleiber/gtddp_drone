//System headers
#include "ros/ros.h"

//User headers
#include "gtddp_drone/state_data.h"
#include "gtddp_drone/state_estimate.h"

//Define program constants
#define MAX_BUFFER 100

/**
 * 
 */
int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "gtddp_state_estimate");

    //Initialize this node
    ros::NodeHandle state_node;

    //Advertise state estimation data
    ros::Publisher state_pub = state_node.advertise<gtddp_drone::state_data>("/gtddp_drone/state_estimate", MAX_BUFFER);

    //Set up the trajectory optimizer
    StateEstimator state_estimation(state_pub);

    //Subscribe to the vicon topic
    ros::Subscriber imu_sub = state_node.subscribe("/ardrone/imu/data", MAX_BUFFER, &StateEstimator::imu_callback, &state_estmation);
    ros::Subscriber nav_sub = state_node.subscribe("/ardrone/navdata", MAX_BUFFER, &StateEstimator::navdata_callback, &state_estimation);
    ros::Subscriber vicon_sub = state_node.subscribe("/vicon/drone_0/odom", MAX_BUFFER, &StateEstimator::vicon_data_callback, &state_estimation);

    //Set up timer for publishing new state data
    //This runs at 100Hz
    //TODO: Make this configurable
    ros::Timer update_timer = state_node.createTimer(ros::Duration(0.01), &StateEstimator::state_estimate_publish, &state_estimation, false);

    //Pump callbacks
    ros::spin();

    return 0;
}