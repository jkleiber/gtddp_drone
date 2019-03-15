#include "gtddp_drone/traj_optimizer.h"


/**
 * 
 */
Optimizer::Optimizer()
{
    //Flag the state data as uninitialized
    this->cur_state_init = false;
    this->goal_state_init = false;

    //Initialize current state size
    this->cur_state.resize(Constants::num_states);
    this->goal_state.resize(Constants::num_states);
}


/**
 * 
 */
Optimizer::Optimizer(ros::Publisher& traj_publisher, ros::Publisher& state_publisher)
{
    this->traj_pub = traj_publisher;
    this->state_pub = state_publisher;
    
    //Flag the state data as uninitialized
    this->cur_state_init = false;
    this->goal_state_init = false;

    //Initialize current state size
    this->cur_state.resize(Constants::num_states);
    this->goal_state.resize(Constants::num_states);
}


/**
 * 
 */
void Optimizer::traj_update_callback(const ros::TimerEvent& time_event)
{
    //Check to make sure current state and target state are initialized
    //If they are, then optimize the current trajectory
    if(this->cur_state_init && this->goal_state_init)
    {
        ddpmain.update(this->cur_state, this->goal_state);
        ddpmain.ddp_loop();

        printf("c\n");

        //Publise the newly optimized trajectory data to the trajectory topic
        traj_pub.publish(this->get_traj_msg(ddpmain.get_x_traj(), ddpmain.get_u_traj(), ddpmain.get_Ku()));
    }
}


/**
 * 
 */
void Optimizer::ground_truth_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
    //Position
    this->cur_state(0) = odom->pose.pose.position.x;
    this->cur_state(1) = odom->pose.pose.position.y;
    this->cur_state(2) = odom->pose.pose.position.z;

    //Orientation
    tf::Pose pose;
    tf::poseMsgToTF(odom->pose.pose, pose);
    tf::Matrix3x3 mat(pose.getRotation());

    //Convert the quaternion to euler angles of yaw, pitch, and roll
    //This should be in radians
    mat.getEulerYPR(this->cur_state(8), this->cur_state(7), this->cur_state(6));
    //mat.getEulerYPR(this->cur_state(5), this->cur_state(4), this->cur_state(3));

    //Linear velocity
    this->cur_state(3) = odom->twist.twist.linear.x;
    this->cur_state(4) = odom->twist.twist.linear.y;
    this->cur_state(5) = odom->twist.twist.linear.z;
    //this->cur_state(6) = odom->twist.twist.linear.x;
    //this->cur_state(7) = odom->twist.twist.linear.y;
    //this->cur_state(8) = odom->twist.twist.linear.z;

    //Angular velocity
    this->cur_state(9) = odom->twist.twist.angular.x;
    this->cur_state(10) = odom->twist.twist.angular.y;
    this->cur_state(11) = odom->twist.twist.angular.z;

    //Set the current state as initialized
    this->cur_state_init = true;

    gtddp_drone_msgs::state_data current_state_dbg;

    for(int i = 0; i < Constants::num_states; ++i)
    {
        current_state_dbg.states[i] = this->cur_state(i);
    }

    this->state_pub.publish(current_state_dbg);
}

/**
 * 
 */
void Optimizer::state_estimate_callback(const tum_ardrone::filter_state::ConstPtr& estimate_event)
{
    //Update the current state from the tum_ardrone
    this->cur_state(0) = estimate_event->x;
    this->cur_state(1) = estimate_event->y;
    this->cur_state(2) = estimate_event->z;

    this->cur_state(3) = estimate_event->dx;
    this->cur_state(4) = estimate_event->dy;
    this->cur_state(5) = estimate_event->dz;

    this->cur_state(6) = estimate_event->roll;
    this->cur_state(7) = estimate_event->pitch;
    this->cur_state(8) = estimate_event->yaw;
    
    this->cur_state(9) = estimate_event->droll;
    this->cur_state(10) = estimate_event->dpitch;
    this->cur_state(11) = estimate_event->dyaw;

    //TODO: Review dynamics
    //this->cur_state(3) = estimate_event->roll;
    //this->cur_state(4) = estimate_event->pitch;
    //this->cur_state(5) = estimate_event->yaw;
    //this->cur_state(6) = estimate_event->dx;
    //this->cur_state(7) = estimate_event->dy;
    //this->cur_state(8) = estimate_event->dz;

    //Set the current state as initialized
    this->cur_state_init = true;
}


/**
 * 
 */
void Optimizer::target_state_callback(const gtddp_drone_msgs::state_data::ConstPtr& target_event)
{
    for(int i = 0; i < Constants::num_states; ++i)
    {
        this->goal_state(i) = target_event->states[i];
    }

    this->goal_state_init = true;
}



/**
 * 
 */
gtddp_drone_msgs::Trajectory Optimizer::get_traj_msg(std::vector<Eigen::VectorXd> x_traj, std::vector<Eigen::VectorXd> u_traj, std::vector<Eigen::MatrixXd> K_traj)
{
    //Declare local variables
    int i;                              //iteration variable
    gtddp_drone_msgs::Trajectory traj_msg;   //trajectory message result

    //Loop through each time step and encode the data into ROS messages
    //Note: the ddp only initializes values from 0 to num_time_steps - 1. Thus, 100 timesteps will yield 99 values
    for(i = 0; i < Constants::num_time_steps - 1; ++i)
    {
        traj_msg.x_traj.push_back(this->get_state_data_msg(x_traj, i));
        traj_msg.u_traj.push_back(this->get_ctrl_data_msg(u_traj, i));
        traj_msg.K_traj.push_back(this->get_gain_data_msg(K_traj, i));
    }

    //Set up the Header of this message (for time tracking)
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.header.frame_id = "/world";

    //Return the generated message
    return traj_msg;
}


/**
 * 
 */
gtddp_drone_msgs::state_data Optimizer::get_state_data_msg(std::vector<Eigen::VectorXd> x_traj, int idx)
{
    //Set up a state data message
    gtddp_drone_msgs::state_data state_msg;

    //Loop through each state variable for this particular state to extract the value
    for(int i = 0; i < Constants::num_states; ++i)
    {
        state_msg.states[i] = (x_traj[idx](i));
    }

    //Return the generated message
    return state_msg;
}


/**
 * 
 */
gtddp_drone_msgs::ctrl_data Optimizer::get_ctrl_data_msg(std::vector<Eigen::VectorXd> u_traj, int idx)
{
    //Set up a state data message
    gtddp_drone_msgs::ctrl_data ctrl_msg;

    //Loop through each state variable for this particular state to extract the value
    for(int i = 0; i < Constants::num_controls_u; ++i)
    {
        ctrl_msg.ctrl[i] = u_traj[idx](i);
    }

    //Return the generated message
    return ctrl_msg;
}


/**
 * 
 */
gtddp_drone_msgs::gain_data Optimizer::get_gain_data_msg(std::vector<Eigen::MatrixXd> K_traj, int idx)
{
    //Set up the gain data message types
    gtddp_drone_msgs::gain_data gain_msg;
    gtddp_drone_msgs::gain_vector gain_row;

    //Declare local variables
    int r, c;

    //Loop through the gain matrix for this particular state
    for(r = 0; r < Constants::num_controls_u; ++r)
    {
        //Clear old row  data
        gain_row.gain_list.clear();

        //Add all the values to the row vector
        for(c = 0; c < Constants::num_states; ++c)
        {
            gain_row.gain_list.push_back(K_traj[idx](r,c));
        }

        //Save the row
        gain_msg.gain_mat.push_back(gain_row);
    }

    //Return the processed gain message
    return gain_msg;
}
