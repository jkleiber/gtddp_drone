
#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <gtddp_drone_msgs/ctrl_data.h>
#include <gtddp_drone_msgs/state_data.h>

#include "gtddp_drone/gtddp_lib/Constants.h"
#include "gtddp_drone/gtddp_lib/systems/CartPole.h"
#include "gtddp_drone/control_calc.h"

// Simulation state
bool sim_started = false;
bool control_empty = false;

// Current state of cart
Eigen::VectorXd cart_state;

// Control publisher
ros::Publisher ctrl_pub;
ros::Publisher stop_pub;

// Set up the control calculation class
// This is hardcoded to always be a simulation
ControlCalculator *control_calc;

// System
CartPole *cart = new CartPole;

// Feedback controller data
int timestep = 0;


/**
 * @brief Update the current known state of the cart-pole system
 *
 * @param state 4D state vector from the cart pole simulator
 */
void cartStateCallback(const gtddp_drone_msgs::state_data::ConstPtr& state)
{
    if(sim_started)
    {
        // Read the state message
        for(int i = 0; i < Constants::num_states; ++i)
        {
            cart_state(i) = state->state[i];
        }

        // Log the state
        //control_calc->log_ground_truth(cart_state);
    }
}


void controlEmptyCallback(const std_msgs::Bool::ConstPtr& bool_msg)
{
    control_empty = bool_msg->data;
}



void controlUpdateCallback(const ros::TimerEvent& event)
{
    TrajectoryPoint traj_pt;
    gtddp_drone_msgs::ctrl_data ctrl_msg;

    // Set ctrl size
    ctrl_msg.ctrl.resize(Constants::num_controls_u);

    // Only get the control update if the trajectory is available and the simulation has started
    if(control_calc->is_traj_available() && sim_started && ctrl_pub.getNumSubscribers() >= 1)
    {
        // Get current control data
        traj_pt = control_calc->pop_traj_point();

        // Log the raw control
        control_calc->log_control(traj_pt.u);

        // Feedback Controller
        traj_pt.u += traj_pt.Ku * (cart_state - traj_pt.x);
        // traj_pt.v += traj_pt.Kv * (cart_state - traj_pt.x);

        // Publish control message
        ctrl_msg.ctrl[0] = traj_pt.u(0); // + traj_pt.v(0);
        ctrl_pub.publish(ctrl_msg);

        // Log the control message
        // control_calc->log_control(traj_pt.u);
    }
    // If the trajectory has been fully read, then stop the simulation
    else if(!control_calc->is_traj_available() && sim_started && control_empty)
    {
        // Tell the simulator to stop updating
        std_msgs::Empty stop_msg;
        stop_pub.publish(stop_msg);

        // Exit
        exit(0);
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "cart_control_node");

    ros::NodeHandle node;

    // Initialize controller
    control_calc = new ControlCalculator(1);
    control_calc->set_system(cart);
    control_calc->control_init();

    // Empty message for starting things up
    std_msgs::Empty start_msg;

    // Load the constants
    ConstantLoader loader(node);

    // Initialize state vector
    cart_state = Eigen::VectorXd::Zero(Constants::num_states);

    // Set up publishers for start signal and control
    ros::Publisher start_pub = node.advertise<std_msgs::Empty>(node.resolveName("/cart/start"), 1);
    stop_pub = node.advertise<std_msgs::Empty>(node.resolveName("/cart/stop"), 1);
    ctrl_pub = node.advertise<gtddp_drone_msgs::ctrl_data>(node.resolveName("/cart/control"), 1);

    // Subscribe to the state estimator
    ros::Subscriber state_sub = node.subscribe(node.resolveName("/cart/state"), 1, &cartStateCallback);

    // Collect control trajectory data from the optimizer
    ros::Subscriber traj_sub = node.subscribe(node.resolveName("/gtddp_drone/trajectory"), 10, &ControlCalculator::trajectory_callback, control_calc);
    ros::Subscriber ctrl_empty_sub = node.subscribe(node.resolveName("/gtddp/control/empty"), 1, &controlEmptyCallback);

    // Run the control update really fast
    ros::Timer ctrl_update_timer = node.createTimer(ros::Duration(0.0009), &controlUpdateCallback);

    // HACK: initialize the optimizer node from here
    ros::Publisher init_pub = node.advertise<std_msgs::Empty>(node.resolveName("/gtddp_drone/start"), 1);

    // Wait for the optimizer to load and then publish
    while (init_pub.getNumSubscribers() < 1)
    {
        sleep(1);
    }
    init_pub.publish(start_msg);

    // Wait for the simulator to boot up and subscribe to this control node
    while(start_pub.getNumSubscribers() < 1)
    {
        sleep(1);
    }

    // Send the simulator start command
    start_pub.publish(start_msg);
    sim_started = true;

    ros::spin();

    return 0;
}
