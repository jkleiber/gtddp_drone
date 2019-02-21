#ifndef CONTROL_CALC_H
#define CONTROL_CALC_H

//System libs
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

//User libs
#include "gtddp_drone/gtddp_lib/Constants.h"

//User msgs
#include <tum_ardrone/filter_state.h>
#include <gtddp_drone/Trajectory.h>

/**
 * 
 */
class ControlCalculator
{
    public:
        ControlCalculator();
        ControlCalculator(ros::Publisher ctrl_sig_pub, ros::Publisher land_pub);

        //Callback functions
        void recalculate_control_callback(const ros::TimerEvent& time_event);
        void ground_truth_callback(const nav_msgs::Odometry::ConstPtr& odom);
        void state_estimate_callback(const tum_ardrone::filter_state::ConstPtr& estimate_event);
        void trajectory_callback(const gtddp_drone::Trajectory::ConstPtr& traj_msg);

    private:
        //Publish control system data to the drone
        ros::Publisher control_signal_pub;
        ros::Publisher landing_pub;
        gtddp_drone::ctrl_data ctrl_command;

        //Store the current trajectory information
        std::vector<Eigen::VectorXd> x_traj;
        std::vector<Eigen::VectorXd> u_traj;
        std::vector<Eigen::MatrixXd> K_traj;

        //Store current state data
        Eigen::VectorXd cur_state;

        //Do time tracking
        int timestep;

        //Flags for initialization
        bool cur_state_init;
        bool traj_init;

};


#endif