#ifndef CONTROL_CALC_H
#define CONTROL_CALC_H

//ROS Libs
#include <ros/ros.h>

//System libs
#include <chrono>
#include <ctime>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <math.h>
#include <pwd.h>
#include <queue>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

//ROS System messages
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

//User libs
#include "gtddp_drone/gtddp_lib/Constants.h"
#include "gtddp_drone/gtddp_lib/Drone.h"
#include "gtddp_drone/gtddp_lib/systems/Quadrotor.h"
#include "gtddp_drone/gtddp_lib/systems/PursuitDrones.h"
#include "gtddp_drone/flight_controller.h"

//User msgs
#include <gtddp_drone_msgs/Trajectory.h>
#include <gtddp_drone_msgs/Status.h>


/**
 *
 */
class ControlCalculator
{
    public:
        ControlCalculator();
        ControlCalculator(ros::Publisher ctrl_sig_pub, ros::Publisher status_pub, int sim_status);

        //Initialization
        void logging_init();

        //Callback functions
        void recalculate_control_callback(const ros::TimerEvent& time_event);
        void state_estimate_callback(const nav_msgs::Odometry::ConstPtr& odom);
        //TODO: rename the state_estimate_callback below to something else
        //void state_estimate_callback(const tum_ardrone::filter_state::ConstPtr& estimate_event);
        void trajectory_callback(const gtddp_drone_msgs::Trajectory::ConstPtr& traj_msg);

        // Test Code
        void open_loop_control(const ros::TimerEvent& time_event);

        //Timer configuration
        void set_timer(ros::Timer& timer);

        ~ControlCalculator();

    private:
        //Flight controller
        FlightController flight_controller;

        //Controller status
        gtddp_drone_msgs::Status ctrl_status;

        //Output format utils
        double angleWrap(double angle);
        double clamp(double val, double min_val, double max_val);

        //Publish control system data to the drone
        ros::Publisher control_signal_pub;
        ros::Publisher status_pub;
        geometry_msgs::Twist ctrl_command;

        //Store the current trajectory information
        DroneTrajectory drone_traj;

        //Store current state data
        Eigen::VectorXd cur_state;

        //Use quadrotor dynamics
        //TODO: change to System class
        Quadrotor quadrotor;

        //Change execution rate for the control callback
        ros::Timer ctrl_timer;

        //Do time tracking
        ros::Time begin_time;
        int timestep;

        //Flags for initialization
        bool cur_state_init;
        bool traj_init;

        //Simulation flag
        bool is_sim;

        std::ofstream ground_truth_data;
        std::ofstream command_data;
};


#endif
