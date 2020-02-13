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

    // Control Constraint DDP Hyperparameters
    double du_converge_dist(2.0);
    double dv_converge_dist(2.0);
    double u0_upper(20), u0_lower(-20);
    double u1_upper(20), u1_lower(-20);
    double u2_upper(20), u2_lower(-20);
    double u3_upper(20), u3_lower(-20);
    double v0_upper(20), v0_lower(-20);
    double v1_upper(20), v1_lower(-20);
    double v2_upper(20), v2_lower(-20);
    double v3_upper(20), v3_lower(-20);

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

    std::string ddp_selector("gtddp");
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
    Constants::ddp_selector = selector;
    selector = "/" + selector;

    // Update the Quadrotor State information
    Constants::num_states = nh.param(selector + "/num_states", 0);
    Constants::num_controls_u = nh.param(selector + "/num_controls_u", 0);
    Constants::num_controls_v = nh.param(selector + "/num_controls_v", 0);

    // Update the DDP learning parameters
    Constants::dt = nh.param<double>(selector + "/dt", 0.001);
    Constants::learning_rate = nh.param<double>(selector + "/learning_rate", 0.0);

    // Update other DDP Hyperparameters
    Constants::num_time_steps = nh.param(selector + "/num_time_steps", 0);
    Constants::num_iterations = nh.param(selector + "/num_iterations", 0);
    Constants::num_long_legs = nh.param(selector + "/num_long_legs", 0);
    Constants::short_iterations = nh.param(selector + "/short_iterations", 0);

    // Update the system dynamics constants
    Constants::m = nh.param<double>(selector + "/mass", 0.436);
    Constants::length = nh.param<double>(selector + "/length", 0.19);
    Constants::Ixx = nh.param<double>(selector + "/Ixx", 0.0045);
    Constants::Iyy = nh.param<double>(selector + "/Iyy", 0.0051);
    Constants::Izz = nh.param<double>(selector + "/Izz", 0.0095);

    // Update the control constraint hyperparameters
    Constants::du_converge_dist = nh.param(selector + "/du_converge_dist", 2.0);
    Constants::dv_converge_dist = nh.param(selector + "/dv_converge_dist", 2.0);

    // Update control constraint limits
    // du
    Constants::u0_upper = nh.param<double>(selector + "/u0_upper", 20);
    Constants::u1_upper = nh.param<double>(selector + "/u1_upper", 20);
    Constants::u2_upper = nh.param<double>(selector + "/u2_upper", 20);
    Constants::u3_upper = nh.param<double>(selector + "/u3_upper", 20);
    Constants::u0_lower = nh.param<double>(selector + "/u0_lower", -20);
    Constants::u1_lower = nh.param<double>(selector + "/u1_lower", -20);
    Constants::u2_lower = nh.param<double>(selector + "/u2_lower", -20);
    Constants::u3_lower = nh.param<double>(selector + "/u3_lower", -20);
    // dv
    Constants::v0_upper = nh.param<double>(selector + "/v0_upper", 20);
    Constants::v1_upper = nh.param<double>(selector + "/v1_upper", 20);
    Constants::v2_upper = nh.param<double>(selector + "/v2_upper", 20);
    Constants::v3_upper = nh.param<double>(selector + "/v3_upper", 20);
    Constants::v0_lower = nh.param<double>(selector + "/v0_lower", -20);
    Constants::v1_lower = nh.param<double>(selector + "/v1_lower", -20);
    Constants::v2_lower = nh.param<double>(selector + "/v2_lower", -20);
    Constants::v3_lower = nh.param<double>(selector + "/v3_lower", -20);
}
