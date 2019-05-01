#ifndef CONTROL_CALC_H
#define CONTROL_CALC_H

//System libs
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <string.h>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

//User libs
#include "gtddp_drone/gtddp_lib/Constants.h"
#include "gtddp_drone/gtddp_lib/Quadrotor.h"

//User msgs
#include <tum_ardrone/filter_state.h>
#include <gtddp_drone_msgs/Trajectory.h>
#include <gtddp_drone_msgs/Status.h>

//Conversion Constants
//TODO: These are simulator values, and may need to be tuned for real flights
#define MAX_FWD_VEL (double)(2.0)       //Max forward velocity (m/s)
#define MAX_SIDE_VEL (double)(2.0)      //Max side velocity (m/s)
#define MAX_YAW_RATE (double)(3.2)      //Max yaw rate (rads / sec)
#define MAX_VERTICAL_VEL (double)(0.5)  //Max vertical speed (m/s)

//PD Controller Constants
#define KP (double)(0.3)
#define KD (double)(0.1)

/**
 * 
 */
class ControlCalculator
{
    public:
        ControlCalculator();
        ControlCalculator(ros::Publisher ctrl_sig_pub, ros::Publisher status_pub);

        //Callback functions
        void recalculate_control_callback(const ros::TimerEvent& time_event);
        void state_estimate_callback(const nav_msgs::Odometry::ConstPtr& odom);
        //TODO: rename the state_estimate_callback below to something else
        //void state_estimate_callback(const tum_ardrone::filter_state::ConstPtr& estimate_event);
        void trajectory_callback(const gtddp_drone_msgs::Trajectory::ConstPtr& traj_msg);

        //Timer configuration
        void set_timer(ros::Timer& timer);

        ~ControlCalculator();

    private:
        //PD Controller
        geometry_msgs::Twist attitude_pd_control();
        geometry_msgs::Twist full_pd_control();
        
        //Controller status
        gtddp_drone_msgs::Status ctrl_status;

        //Output format utils
        double angleWrap(double angle);

        //Publish control system data to the drone
        ros::Publisher control_signal_pub;
        ros::Publisher status_pub;
        geometry_msgs::Twist ctrl_command;

        //Store the current trajectory information
        std::vector<Eigen::VectorXd> x_traj;
        std::vector<Eigen::VectorXd> u_traj;
        std::vector<Eigen::MatrixXd> K_traj;

        //Store current state data
        Eigen::VectorXd cur_state;

        //Use quadrotor dynamics
        Quadrotor quadrotor;

        //Change execution rate for the control callback
        ros::Timer ctrl_timer;

        //Do time tracking
        int timestep;

        //Flags for initialization
        bool cur_state_init;
        bool traj_init;

        std::ofstream ground_truth_data;
        std::ofstream target_data;
};


#endif