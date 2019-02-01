#include "traj_optimizer.h"

Optimizer::Optimizer()
{}

Optimizer::Optimizer(ros::Publisher& publisher)
{
    this->traj_pub = publisher;
}

void Optimizer::traj_update_callback(const ros::TimerEvent& time_event)
{

}

void Optimizer::state_estimate_callback(const std_msgs::Header& estimate_event)
{

}

void Optimizer::target_state_callback(const std_msgs::Header& target_event)
{
    
}