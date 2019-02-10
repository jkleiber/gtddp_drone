#include "gtddp_drone/control_calc.h"

ControlCalculator::ControlCalculator()
{

}



ControlCalculator::ControlCalculator(ros::Publisher ctrl_sig_pub)
{

}



void ControlCalculator::recalculate_control_callback(const ros::TimerEvent& time_event)
{
    //Somehow determine the curretn time and where that is in the traj

    //Calculate u(t)

    //Publish u(t) to the control signal topic
}



void ControlCalculator::trajectory_callback(const gtddp_drone::Trajectory::ConstPtr& traj_msg)
{
    //Get the components of the message
    const std::vector<gtddp_drone::state_data> &x_data = traj_msg->x_traj;
    const std::vector<gtddp_drone::ctrl_data> &u_data = traj_msg->u_traj;
    const std::vector<gtddp_drone::gain_data> &K_data = traj_msg->K_traj;

    //Parse the message to get the trajectory
    
}


