#ifndef TRAJ_OPTIMIZE_H
#define TRAJ_OPTIMIZE_H

#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <vector>

#include "gtddp_lib/Constants.h"
#include "gtddp_lib/DDP_main_mm.h"

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
        Eigen::VectorXd cur_state(num_states);
        Eigen::VectorXd goal_state(num_states);

        //Data parsing for sending messages
        gtddp_drone::Trajectory get_traj_msg(std::vector<Eigen::VectorXd> x_traj, std::vector<Eigen::VectorXd> u_traj, std::vector<Eigen::MatrixXd> K_traj);
        gtddp_drone::state_data get_state_data_msg(std::vector<Eigen::VectorXd> x_traj, int idx);
        gtddp_drone::ctrl_data get_ctrl_data_msg(std::vector<Eigen::VectorXd> u_traj, int idx);
        gtddp_drone::gain_data get_gain_data_msg(std::vector<Eigen::VectorXd> u_traj, int idx);
};

#endif