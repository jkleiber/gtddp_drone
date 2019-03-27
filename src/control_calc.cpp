#include "gtddp_drone/control_calc.h"

/**
 * 
 */
ControlCalculator::ControlCalculator()
{
    //Initialize current state size
    this->cur_state.resize(Constants::num_states);

    //Reset flags
    this->cur_state_init = false;
    this->traj_init = false;

    //Set timestep to t0
    this->timestep = -1;
}


/**
 * 
 */
ControlCalculator::ControlCalculator(ros::Publisher ctrl_sig_pub, ros::Publisher land_pub)
{
    //Control signal to publish to AR Drone
    this->control_signal_pub = ctrl_sig_pub;

    //For landing the drone, use the landing Publisher
    this->landing_pub = land_pub;

    //Initialize current state size
    this->cur_state.resize(Constants::num_states);

    //Reset flags
    this->cur_state_init = false;
    this->traj_init = false;

    //Set timestep to t0
    this->timestep = -1;
}


/**
 * 
 */
void ControlCalculator::recalculate_control_callback(const ros::TimerEvent& time_event)
{
    //Only output to the control topic if the localization has happened and the trajectory has been built
    //Also only output if the timestep is in bounds
    if(this->cur_state_init && this->traj_init
    && timestep >= 0 && timestep < this->x_traj.size())
    {
        printf("[");
        for(int i = 0; i < Constants::num_states; ++i)
        {
            printf("%f ", this->x_traj[timestep](i));
        }
        printf("]\n");
        
        /* Form the control message */
        //Pitch (move forward) (theta)
        this->ctrl_command.linear.x = this->x_traj[timestep](7);// / MAX_PITCH_ANGLE;
        
        //Roll (move side to side) (-phi)
        this->ctrl_command.linear.y = -this->x_traj[timestep](6);// / MAX_ROLL_ANGLE;
        
        //Yaw rate (how fast to spin) (r)
        this->ctrl_command.angular.z = this->x_traj[timestep](11) / MAX_YAW_RATE;
    
        //Vertical speed (how fast to move upward) (z dot)
        this->ctrl_command.linear.z = this->x_traj[timestep](5) / MAX_VERTICAL_VEL;
    
        //Increment the timestep
        this->timestep++;

        //Publish u(t) to the control signal topic
        this->control_signal_pub.publish(this->ctrl_command);
    }
    //TODO: will this crash a real-life drone? Currently used to help with laptop simulations
    else if(this->timestep >= 0)
    {
        this->ctrl_command.linear.x = 0;
        this->ctrl_command.linear.y = 0;
        this->ctrl_command.linear.z = 0;
        this->ctrl_command.angular.x = 0;
        this->ctrl_command.angular.y = 0;
        this->ctrl_command.angular.z = 0;
    }

    //TODO: see above todo. Will this crash a real drone?
    //Publish u(t) to the control signal topic
    //this->control_signal_pub.publish(this->ctrl_command);
}


//TODO: add PD controller here. add control policy back
/**
 * 
 */
void ControlCalculator::ground_truth_callback(const nav_msgs::Odometry::ConstPtr& odom)
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

    //Publish control data from PD controller
    control_signal_pub.publish(attitude_pd_control(this->cur_state(6), this->cur_state(9), this->cur_state(7), this->cur_state(10)));
}



/**
 * 
 */
void ControlCalculator::state_estimate_callback(const tum_ardrone::filter_state::ConstPtr& estimate_event)
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
void ControlCalculator::trajectory_callback(const gtddp_drone_msgs::Trajectory::ConstPtr& traj_msg)
{
    //Declare local variables
    int i;          //iteration variable
    int num_events; //number of events in the message
    int t;          //time iteration variable
    int r, c;       //rows and columns of gain matrix

    //Get the components of the message
    const std::vector<gtddp_drone_msgs::state_data> &x_data = traj_msg->x_traj;
    const std::vector<gtddp_drone_msgs::ctrl_data> &u_data = traj_msg->u_traj;
    const std::vector<gtddp_drone_msgs::gain_data> &K_data = traj_msg->K_traj;

    //Find the number of events
    num_events = x_data.size();

    //Clear the trajectory vectors
    this->x_traj.clear();
    this->u_traj.clear();
    this->K_traj.clear();

    //Create temporary vectors for the trajectory data
    Eigen::VectorXd x_traj_dat(Constants::num_states);
    Eigen::VectorXd u_traj_dat(Constants::num_controls_u);
    Eigen::MatrixXd K_traj_dat(Constants::num_controls_u, Constants::num_states);

    //Parse the message to get the trajectory
    for(t = 0; t < num_events; ++t)
    {
        //x_traj
        for(i = 0; i < Constants::num_states; ++i)
        {
            x_traj_dat(i) = x_data[t].states[i];
        }

        //u_traj
        for(i = 0; i < Constants::num_controls_u; ++i)
        {
            u_traj_dat(i) = u_data[t].ctrl[i];
        }

        //K_traj
        //Go through rows and columns of the message
        for(r = 0; r < Constants::num_controls_u; ++r)
        {
            for(c = 0; c < Constants::num_states; ++c)
            {
                K_traj_dat(r,c) = K_data[t].gain_mat[r].gain_list[c];
            }
        }

        //Push the new data into storage
        this->x_traj.push_back(x_traj_dat);
        this->u_traj.push_back(u_traj_dat);
        this->K_traj.push_back(K_traj_dat);
    }

    //Forward propagate controls to find the correct output
    quadrotor.feedforward_controls(this->cur_state, this->u_traj, this->K_traj, this->x_traj);

    //Reset the timestep variable to t0
    timestep = 0;

    //The trajectory is now initialized!
    this->traj_init = true;
}



geometry_msgs::Twist ControlCalculator::attitude_pd_control(const double phi, const double theta, const double phi_dot, const double theta_dot)
{
    //Phi PD control
    double phi_error = (phi - this->ctrl_command.linear.y);
    double pd_phi = (KP * phi_error) + (KD * phi_dot);

    //Theta PD control
    double theta_error = (theta - this->ctrl_command.linear.x);
    double pd_theta = (KP * theta_error) + (KD * theta_dot);

    //Make the augmented message
    geometry_msgs::Twist pd_output;
    pd_output = this->ctrl_command;

    //Modify original output to handle the PD alterations
    pd_output.linear.x = pd_theta / MAX_PITCH_ANGLE;
    pd_output.linear.y = pd_phi / MAX_ROLL_ANGLE;

    return pd_output;
}