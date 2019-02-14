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
}


/**
 * 
 */
void ControlCalculator::recalculate_control_callback(const ros::TimerEvent& time_event)
{
    //Increment the timestep
    this->timestep++;

    //Only output to the control topic if the localization has happened and the trajectory has been built
    //Also only output if the timestep is in bounds
    if(this->cur_state_init && this->traj_init
    && timestep >= 0 && timestep < u_traj.size())
    {
        //TODO: Make K_traj the right dimensions for multiplication
        //Calculate u(t)
        Eigen::VectorXd ctrl_vector = u_traj[timestep] + K_traj[timestep] * (x_traj[timestep] - cur_state);

        //Form the control message
        this->ctrl_command.ctrl[0] = ctrl_vector(0);
        this->ctrl_command.ctrl[1] = ctrl_vector(1);
        this->ctrl_command.ctrl[2] = ctrl_vector(2);
        this->ctrl_command.ctrl[3] = ctrl_vector(3);

        //Publish u(t) to the control signal topic
        this->control_signal_pub.publish(this->ctrl_command);
    }
}


/**
 * 
 */
void ControlCalculator::state_estimate_callback(const tum_ardrone::filter_state::ConstPtr& estimate_event)
{
    //Set the current state as initialized
    this->cur_state_init = true;

    //Update the current state from the tum_ardrone
    this->cur_state(0) = estimate_event->x;
    this->cur_state(1) = estimate_event->y;
    this->cur_state(2) = estimate_event->z;
    this->cur_state(3) = estimate_event->roll;
    this->cur_state(4) = estimate_event->pitch;
    this->cur_state(5) = estimate_event->yaw;
    this->cur_state(6) = estimate_event->dx;
    this->cur_state(7) = estimate_event->dy;
    this->cur_state(8) = estimate_event->dz;
    this->cur_state(9) = estimate_event->droll;
    this->cur_state(10) = estimate_event->dpitch;
    this->cur_state(11) = estimate_event->dyaw;
}


/**
 * 
 */
void ControlCalculator::trajectory_callback(const gtddp_drone::Trajectory::ConstPtr& traj_msg)
{
    //Declare local variables
    int i;          //iteration variable
    int num_events; //number of events in the message
    int t;          //time iteration variable
    int r, c;       //rows and columns of gain matrix

    //The trajectory is now initialized!
    this->traj_init = true;

    //Get the components of the message
    const std::vector<gtddp_drone::state_data> &x_data = traj_msg->x_traj;
    const std::vector<gtddp_drone::ctrl_data> &u_data = traj_msg->u_traj;
    const std::vector<gtddp_drone::gain_data> &K_data = traj_msg->K_traj;

    //Find the number of events
    num_events = x_data.size();

    //Clear the trajectory vectors
    this->x_traj.clear();
    this->u_traj.clear();
    this->K_traj.clear();

    //Create temporary vectors for the trajectory data
    Eigen::VectorXd x_traj_dat(Constants::num_states);
    Eigen::VectorXd u_traj_dat(Constants::num_controls_u);
    Eigen::MatrixXd K_traj_dat(Constants::num_controls_u, Constants::num_controls_u);

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
            for(c = 0; c < Constants::num_controls_u; ++c)
            {
                K_traj_dat(r,c) = K_data[t].gain_mat[r].gain_list[c];
            }
        }

        //Push the new data into storage
        this->x_traj.push_back(x_traj_dat);
        this->u_traj.push_back(u_traj_dat);
        this->K_traj.push_back(K_traj_dat);
    }

    //Reset the timestep variable to -1 for calculation
    timestep = -1;
}
