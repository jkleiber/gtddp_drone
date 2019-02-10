#ifndef CONTROL_CALC_H
#define CONTROL_CALC_H

#include "ros/ros.h"

/**
 * 
 */
class ControlCalculator
{
    public:
        ControlCalculator();
        ControlCalculator(ros::Publisher ctrl_sig_pub);

        //Callback functions
        void recalculate_control_callback(const ros::TimerEvent& time_event);
        void trajectory_callback(const gtddp_drone::Trajectory::ConstPtr& traj_msg);

    private:
        //Publish control system data to the drone
        ros::Publisher control_signal_pub;

};


#endif