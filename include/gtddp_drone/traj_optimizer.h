#ifndef TRAJ_OPTIMIZE_H
#define TRAJ_OPTIMIZE_H

//System libs
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

//C++ libs
#include <chrono>
#include <ctime> 
#include <fstream>
#include <iostream>
#include <pwd.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

//User defined libs
#include "gtddp_drone/gtddp_lib/Constants.h"
#include "gtddp_drone/gtddp_lib/DDP_main_mm.h"

//ROS msgs
#include <std_msgs/Empty.h>
#include <tum_ardrone/filter_state.h>

//User defined ROS msgs
#include <gtddp_drone_msgs/Trajectory.h>
#include <gtddp_drone_msgs/state_data.h>
#include <gtddp_drone_msgs/ctrl_data.h>
#include <gtddp_drone_msgs/gain_data.h>
#include <gtddp_drone_msgs/gain_vector.h>
#include <gtddp_drone_msgs/Status.h>

//User defined ROS srv
#include <gtddp_drone_msgs/target.h>

class Optimizer 
{
    public:
        //Constructors
        Optimizer();
        ~Optimizer();
        Optimizer(ros::Publisher& traj_publisher, ros::Publisher& state_publisher, ros::Publisher& init_publisher, ros::ServiceClient& target_client);
        
        //Logging
        void logging_init();

        //Callback functions
        void traj_update_callback(const ros::TimerEvent& time_event);
        void state_estimate_callback(const nav_msgs::Odometry::ConstPtr& odom);
        //TODO: rename the state_estimate_callback below to something else
        //void state_estimate_callback(const tum_ardrone::filter_state::ConstPtr& estimate_event);
        void status_callback(const gtddp_drone_msgs::Status::ConstPtr& status);
        void init_optimizer(const std_msgs::Empty::ConstPtr& init_msg);

        //Target state helpers
        void target_state_decode(const gtddp_drone_msgs::state_data& target_event);

    private:
        //Publisher for the trajectory data
        ros::Publisher traj_pub;

        //Publisher for state estimate (simulation debugging)
        ros::Publisher state_pub;

        //Publisher for initial conditions
        ros::Publisher init_pub;

        //ServiceClient for the target data
        ros::ServiceClient target_client;

        //Optimizer initialization flag
        bool initialized;

        //Track the controller's state
        int ctrl_status;

        //Save callback data here
        Eigen::VectorXd cur_state;
        gtddp_drone_msgs::state_data current_state;
        Eigen::VectorXd goal_state;
        Eigen::VectorXd last_goal_state;
        bool cur_state_init;
        bool last_goal_state_init;

        //Save DDP loop state in class variable
        DDP_main_mm ddpmain;

        //Data parsing for sending messages
        gtddp_drone_msgs::Trajectory get_traj_msg(std::vector<Eigen::VectorXd> x_traj, std::vector<Eigen::VectorXd> u_traj, std::vector<Eigen::MatrixXd> K_traj);
        gtddp_drone_msgs::state_data get_state_data_msg(std::vector<Eigen::VectorXd> x_traj, int idx);
        gtddp_drone_msgs::ctrl_data get_ctrl_data_msg(std::vector<Eigen::VectorXd> u_traj, int idx);
        gtddp_drone_msgs::gain_data get_gain_data_msg(std::vector<Eigen::MatrixXd> K_traj, int idx);
        
        //Logging
        std::ofstream init_data;
};

#endif