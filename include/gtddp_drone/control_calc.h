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
        ControlCalculator(ros::Publisher ctrl_sig_pub, int sim_status);
        ControlCalculator(ros::Publisher ctrl_sig_pub, int sim_status, bool is_pursuit, ros::Publisher ctrl_sig_pub_2);

        //Initialization
        void logging_init();

        // Callback functions
        // Drone state estimate(s)
        void state_estimate_callback(const nav_msgs::Odometry::ConstPtr& odom);
        void state_estimate_callback_2(const nav_msgs::Odometry::ConstPtr& odom);
        // Drone controller
        void recalculate_control_callback(const ros::TimerEvent& time_event);
        // Trajectory handler
        void trajectory_callback(const gtddp_drone_msgs::Trajectory::ConstPtr& traj_msg);

        // Test Code
        void open_loop_control(const ros::TimerEvent& time_event);

        //Timer configuration
        void set_timer(ros::Timer& timer);

        ~ControlCalculator();

    private:
        bool is_pursuit;

        //Flight controllers
        FlightController flight_controller;
        FlightController flight_controller_2;

        //Output format utils
        double angleWrap(double angle);
        double clamp(double val, double min_val, double max_val);

        //Publish control system data to the drone
        ros::Publisher control_signal_pub;
        ros::Publisher control_signal_pub_2;
        ros::Publisher status_pub;
        geometry_msgs::Twist ctrl_command;
        geometry_msgs::Twist ctrl_command_2;

        //Store the current trajectory information
        DroneTrajectory drone_traj;

        //Store current state data
        Eigen::VectorXd cur_state;

        //Use specified dynamics
        Drone *drone;

        //Change execution rate for the control callback
        ros::Timer ctrl_timer;

        //Do time tracking
        ros::Time begin_time;
        int timestep;

        //Flags for initialization
        bool cur_state_init;
        bool cur_state_init_2;
        bool traj_init;

        //Simulation flag
        bool is_sim;

        std::ofstream ground_truth_data;
        std::ofstream command_data;


};


#endif
