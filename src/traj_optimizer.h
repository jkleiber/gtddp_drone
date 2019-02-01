#ifndef TRAJ_OPTIMIZE_H
#define TRAJ_OPTIMIZE_H

#include "ros/ros.h"

class Optimizer 
{
    public:
        //Constructors
        Optimizer();
        Optimizer(ros::Publisher& publisher);
        
        //Callback functions
        void traj_update_callback(const ros::TimerEvent& time_event);
        void state_estimate_callback(const std_msgs::Header& estimate_event);
        void target_state_callback(const std_msgs::Header& target_event);

    private:
        //Publisher for the trajectory data
        ros::Publisher traj_pub;

        //Save callback data here

};

#endif