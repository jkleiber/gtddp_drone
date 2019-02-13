#include "gtddp_drone/state_estimate.h"

/**
 * 
 */
StateEstimator::StateEstimator()
{
    this->imu_init = false;
    this->navdata_init = false;
    this->vicon_init = false;
}


/**
 * 
 */
StateEstimator::StateEstimator(ros::Publisher state_pub)
{
    this->imu_init = false;
    this->navdata_init = false;
    this->vicon_init = false;
    this->state_pub = state_pub;
}


/**
 * 
 */
void StateEstimator::vicon_data_callback(const vicon::Subject::ConstPtr& estimate_event)
{
    //Find the position and orientation of the drone in space
    const geometry_msgs::Point drone_pos = estimate_event->position;
    const geometry_msgs::Quaternion drone_orient = estimate_event->orientation;

    //Process these values into the current state vector
    this->cur_state.states[0] = drone_pos.x;    //x position
    this->cur_state.states[1] = drone_pos.y;    //y position
    this->cur_state.states[2] = drone_pos.z;    //z position

    //Vicon data has been initialized!
    this->vicon_init = true;
}


/**
 * 
 */
void StateEstimator::navdata_callback(const ardrone_autonomy::Navdata::ConstPtr& navdata)
{
    //Update appropriate current state vector vars
    //Rotation
    this->cur_state.states[3] = navdata->rotX;  //x axis rotation
    this->cur_state.states[4] = navdata->rotY;  //y axis rotation
    this->cur_state.states[5] = navdata->rotZ;  //z axis rotation

    //Velocity (requires conversion from mm/sec to m/sec)
    this->cur_state.states[6] = navdata->vx / 1000;    //x velocity
    this->cur_state.states[7] = navdata->vy / 1000;    //y velocity
    this->cur_state.states[8] = navdata->vz / 1000;    //z velocity

    //First navdata packet has been received
    this->navdata_init = true;
}


/**
 * 
 */
void StateEstimator::imu_callback(const sensor_msgs::Imu::ConstPtr& imu_data)
{
    //Update the angular velocity variables in the state
    this->cur_state.states[9] = imu_data->angular_velocity.x;   //x angular velocity
    this->cur_state.states[10] = imu_data->angular_velocity.y;  //y angular velocity
    this->cur_state.states[11] = imu_data->angular_velocity.z;  //z angular velocity

    //IMU has been initialized
    this->imu_init = true;
}


/**
 * 
 */
void StateEstimator::state_estimate_publish()
{
    //Only publish data if we have initialized the current state completely
    //i.e. IMU, Navdata, and Vicon data need to have been received at least once
    if(this->imu_init && this->navdata_init && this->vicon_init)
    {
        //Publish current state
        this->state_pub.publish(this->cur_state);
    }
}