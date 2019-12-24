#include "gtddp_drone/gtddp_lib/Constants.h"

namespace Constants {

    /*
        IMPORTANT - do not name any constants with name:
        l, L, g, G, q_0, q_u, q_uu, q_ux, q_x, q_xx
        R, Q_x, Q_f, A, B, x_0, x_target, cost, V, V_x, V_xx
    */

    // Universal Constant ints
    // You can add, but do not change
    extern const double pi(std::atan(1)*4);
    extern const double grav(9.81);


    //Quadrotor system
    int num_states(12);
    int num_controls_u(4);
    int num_controls_v(4);

    // DDP Hyperparameters
    int num_time_steps(501);
    int num_iterations(60);
    int num_long_legs(80);
    int short_iterations(20);

    // DDP Constant doubles
    // Change as necessary, do not remove
    double dt(0.001);
    double learning_rate(0.05);

    // System Constant doubles
    double m(0.436);
    double length(0.19);
    double Ixx(0.0045);
    double Iyy(0.0051);
    double Izz(0.0095);

}


ConstantLoader::ConstantLoader(){}

/**
 * @brief Update all the constants in the Constants namespace
 *
 * @param nh NodeHandle for accessing the ROS parameter server
 */
ConstantLoader::ConstantLoader(ros::NodeHandle nh)
{
    // Choose which constants to use based on the type of motion planning selected
    std::string selector = "gtddp";
    selector = nh.param("/ddp_select", selector);
    selector = "/" + selector;

    // Update the Quadrotor State information
    Constants::num_states = nh.param(selector + "/num_states", 0);
    Constants::num_controls_u = nh.param(selector + "/num_controls_u", 0);
    Constants::num_controls_v = nh.param(selector + "/num_controls_v", 0);

    // Update the DDP learning parameters
    Constants::dt = nh.param(selector + "/dt", 0.001);
    Constants::learning_rate = nh.param(selector + "/learning_rate", 0.0);

    // Update other DDP Hyperparameters
    Constants::num_time_steps = nh.param(selector + "/num_time_steps", 0);
    Constants::num_iterations = nh.param(selector + "/num_iterations", 0);
    Constants::num_long_legs = nh.param(selector + "/num_long_legs", 0);
    Constants::short_iterations = nh.param(selector + "/short_iterations", 0);

    // Update the system dynamics constants
    Constants::m = nh.param(selector + "/mass", 0.436);
    Constants::length = nh.param(selector + "/length", 0.19);
    Constants::Ixx = nh.param(selector + "/Ixx", 0.0045);
    Constants::Iyy = nh.param(selector + "/Iyy", 0.0051);
    Constants::Izz = nh.param(selector + "/Izz", 0.0095);
}
