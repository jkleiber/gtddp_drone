#include "gtddp_drone/control_calc.h"

ControlCalculator::ControlCalculator()
{

}



ControlCalculator::ControlCalculator(ros::Publisher ctrl_sig_pub)
{
    this->control_signal_pub = ctrl_sig_pub;
}



void ControlCalculator::recalculate_control_callback(const ros::TimerEvent& time_event)
{
    //Somehow determine the curretn time and where that is in the traj

    //Calculate u(t)

    //Publish u(t) to the control signal topic
}


//TODO: if necessary, consider renaming this and its partner callback in optimizer to be the vicon_estimate
//in the future it might be necessary to handle multiple callbacks to estimate the state
//such as if we are reading sensor data or doing EKF stuff.
//Then it will come full circle and we will create a node to handle state estimation
//TODO: consider making a separate package and node for state estimation (drone_state_estimate)
void state_estimate_callback(const vicon::Subject::ConstPtr& estimate_event)
{
    //TODO: do the same thing here as in the traj_optimizer
    //TODO: consider a common function that can return the same data
}



void ControlCalculator::trajectory_callback(const gtddp_drone::Trajectory::ConstPtr& traj_msg)
{
    //Get the components of the message
    const std::vector<gtddp_drone::state_data> &x_data = traj_msg->x_traj;
    const std::vector<gtddp_drone::ctrl_data> &u_data = traj_msg->u_traj;
    const std::vector<gtddp_drone::gain_data> &K_data = traj_msg->K_traj;

    //Parse the message to get the trajectory
    
}


