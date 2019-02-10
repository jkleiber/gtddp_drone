#ifndef CONTROL_CALC_H
#define CONTROL_CALC_H

//System libs
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>

//User libs
#include "gtddp_drone/gtddp_lib/Constants.h"

//ROS msgs
#include <gtddp_drone/Trajectory.h>

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

        //Store the current trajectory information
        std::vector<Eigen::VectorXd> x_traj;
        std::vector<Eigen::VectorXd> u_traj;
        std::vector<Eigen::MatrixXd> K_traj;

};


#endif