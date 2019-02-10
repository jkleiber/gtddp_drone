#include "gtddp_drone/traj_optimizer.h"

Optimizer::Optimizer()
{
    //Set state vectors to uninitialized
    this->cur_state.quiet_NaN();
    this->goal_state.quiet_NaN();
}

Optimizer::Optimizer(ros::Publisher& publisher)
{
    this->traj_pub = publisher;

    //Set state vectors to uninitialized
    this->cur_state.quiet_NaN();
    this->goal_state.quiet_NaN();
}

/**
 * 
 */
void Optimizer::traj_update_callback(const ros::TimerEvent& time_event)
{
    //Check to make sure current state and target state are initialized
    //If they are, then optimize the current trajectory
    if(!(this->cur_state.hasNaN() || this->goal_state.hasNaN()))
    {
        DDP_main_mm ddpmain(this->cur_state, this->goal_state);
        ddpmain.ddp_loop();

        //Publise the newly optimized trajectory data to the trajectory topic
        traj_pub.publish(this->get_traj_msg(ddpmain.get_x_traj(), ddpmain.get_u_traj(), ddpmain.get_Ku()));
    }
}

void Optimizer::state_estimate_callback(const tf2_msgs::TFMessage::ConstPtr& estimate_event)
{

}

void Optimizer::target_state_callback(const std_msgs::Header& target_event)
{

}


gtddp_drone::Trajectory Optimizer::get_traj_msg(std::vector<Eigen::VectorXd> x_traj, std::vector<Eigen::VectorXd> u_traj, std::vector<Eigen::MatrixXd> K_traj)
{
    //Declare local variables
    int i;                              //iteration variable
    gtddp_drone::Trajectory traj_msg;   //trajectory message result

    //Loop through each time step and encode the data into ROS messages
    for(i = 0; i < num_time_steps; ++i)
    {
        traj_msg.state_data.push_back(this->get_state_data_msg(x_traj), i);
        traj_msg.ctrl_data.push_back(this->get_ctrl_data_msg(u_traj), i);
        traj_msg.gain_data.push_back(this->get_gain_data_msg(K_traj), i);
    }

    //Set up the Header of this message (for time tracking)
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.header.frame_id = "/world";

    //Return the generated message
    return traj_msg;
}

gtddp_drone::state_data Optimizer::get_state_data_msg(std::vector<Eigen::VectorXd> x_traj, int idx)
{
    //Set up a state data message
    gtddp_drone::state_data state_msg;

    //Loop through each state variable for this particular state to extract the value
    for(int i = 0; i < num_states; ++i)
    {
        state_msg.push_back(x_traj[idx](i));
    }

    //Return the generated message
    return state_msg;
}

gtddp_drone::ctrl_data Optimizer::get_ctrl_data_msg(std::vector<Eigen::VectorXd> u_traj, int idx)
{
    //Set up a state data message
    gtddp_drone::ctrl_data ctrl_msg;

    //Loop through each state variable for this particular state to extract the value
    for(int i = 0; i < num_controls_u; ++i)
    {
        ctrl_msg.push_back(u_traj[idx](i));
    }

    //Return the generated message
    return ctrl_msg;
}

gtddp_drone::gain_data Optimizer::get_gain_data_msg(std::vector<Eigen::MatrixXd> u_traj, int idx)
{
    //Set up the gain data message types
    gtddp_drone::gain_data gain_msg;
    gtddp_drone::gain_vector gain_row;

    //Declare local variables
    int r, c;

    //Loop through the gain matrix for this particular state
    for(r = 0; r < num_controls_u; ++r)
    {
        //Clear old row  data
        gain_row.clear();

        //Add all the values to the row vector
        for(c = 0; c < num_controls_u; ++c)
        {
            gain_row.push_back(K_traj[idx](r,c));
        }

        //Save the row
        gain_msg.push_back(gain_row);
    }

    //Return the processed gain message
    return gain_msg;
}