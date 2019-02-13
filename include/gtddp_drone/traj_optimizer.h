#ifndef TRAJ_OPTIMIZE_H
#define TRAJ_OPTIMIZE_H

//System libs
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <vector>

//User defined libs
#include "gtddp_drone/gtddp_lib/Constants.h"
#include "gtddp_drone/gtddp_lib/DDP_main_mm.h"

//ROS msgs
#include <vicon/Subject.h>
#include <vicon/Markers.h>
#include <vicon/Marker.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>

//User defined ROS msgs
#include <gtddp_drone/Trajectory.h>
#include <gtddp_drone/state_data.h>
#include <gtddp_drone/ctrl_data.h>
#include <gtddp_drone/gain_data.h>
#include <gtddp_drone/gain_vector.h>


class Optimizer 
{
    public:
        //Constructors
        Optimizer();
        Optimizer(ros::Publisher& publisher);
        
        //Callback functions
        void traj_update_callback(const ros::TimerEvent& time_event);
        void state_estimate_callback(const gtddp_drone::state_data::ConstPtr& estimate_event);
        void target_state_callback(const gtddp_drone::state_data::ConstPtr& target_event);

    private:
        //Publisher for the trajectory data
        ros::Publisher traj_pub;

        //Save callback data here
        Eigen::VectorXd cur_state;
        Eigen::VectorXd goal_state;
        bool cur_state_init;
        bool goal_state_init;

        //Data parsing for sending messages
        gtddp_drone::Trajectory get_traj_msg(std::vector<Eigen::VectorXd> x_traj, std::vector<Eigen::VectorXd> u_traj, std::vector<Eigen::MatrixXd> K_traj);
        gtddp_drone::state_data get_state_data_msg(std::vector<Eigen::VectorXd> x_traj, int idx);
        gtddp_drone::ctrl_data get_ctrl_data_msg(std::vector<Eigen::VectorXd> u_traj, int idx);
        gtddp_drone::gain_data get_gain_data_msg(std::vector<Eigen::MatrixXd> K_traj, int idx);
};

#endif