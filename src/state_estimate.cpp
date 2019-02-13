#include "gtddp_drone/state_estimate.h"

/**
 * 
 */
StateEstimator::StateEstimator()
{
    vicon_init = false;
}


/**
 * 
 */
StateEstimator::StateEstimator(ros::Publisher state_pub)
{
    vicon_init = false;
    this->state_pub = state_pub;
}


/**
 * 
 */
void StateEstimator::vicon_data_callback(const vicon::Subject::ConstPtr& estimate_event)
{
    //Vicon data has been initialized!
    vicon_init = true;

    //Find the position and orientation of the drone in space
    const geometry_msgs::Point drone_pos = estimate_event->position;
    const geometry_msgs::Quaternion drone_orient = estimate_event->orientation;

    //Process these values into a vector
}


/**
 * 
 */
void StateEstimator::navdata_callback(const ardrone_autonomy::Navdata::ConstPtr& navdata)
{
    
}


/**
 * 
 */
void StateEstimator::state_estimate_publish()
{
    //Only publish data if we have received vicon data
    if(this->vicon_init)
    {
        //Publish current state
        this->state_pub.publish(this->cur_state);
    }
}