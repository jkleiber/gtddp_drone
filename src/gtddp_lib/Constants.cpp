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
    Eigen::VectorXd u_upper(4), u_lower(4), v_upper(4), v_lower(4);

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

    // Control initial conditions
    Eigen::VectorXd u_hover(4), v_hover(4);

    // Cost function parameters
    Eigen::MatrixXd Ru(4,4), Rv(4,4), Q(12,12);
    double Qx_multiplier(0);

    // Should this be control-constrained (pursuit)
    bool pursuit_constrained(false);

    // How many traj points to read per loop from offline
    int offline_traj_batch_size(300);

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
    Constants::num_states = nh.param("/num_states", 0);
    Constants::num_controls_u = nh.param("/num_controls_u", 0);
    Constants::num_controls_v = nh.param("/num_controls_v", 0);

    // Update the DDP learning parameters
    Constants::dt = nh.param<double>("/dt", 0.001);
    Constants::learning_rate = nh.param<double>("/learning_rate", 0.0);

    // Update other DDP Hyperparameters
    Constants::num_time_steps = nh.param("/num_time_steps", 0);
    Constants::num_iterations = nh.param("/num_iterations", 0);
    Constants::num_long_legs = nh.param("/num_long_legs", 0);
    Constants::short_iterations = nh.param("/short_iterations", 0);

    // Update the system dynamics constants
    Constants::m = nh.param<double>("/mass", 0.436);
    Constants::length = nh.param<double>("/length", 0.19);
    Constants::Ixx = nh.param<double>("/Ixx", 0.0045);
    Constants::Iyy = nh.param<double>("/Iyy", 0.0051);
    Constants::Izz = nh.param<double>("/Izz", 0.0095);

    // Update the control constraint hyperparameters
    Constants::du_converge_dist = nh.param("/du_converge_dist", 2.0);
    Constants::dv_converge_dist = nh.param("/dv_converge_dist", 2.0);

    // Temporary vector for param loading
    std::vector<double> tmp_vector;

    // Update control constraint limits
    // du
    // upper
    if(!nh.getParam("/u_upper", tmp_vector))
    {
        ROS_WARN("Warning: unable to load upper constraint for u_traj");
    }
    else
    {
        for (int i = 0; i < Constants::num_controls_u; ++i)
        {
            Constants::u_upper(i) = tmp_vector[i];
        }
    }
    //lower
    if(!nh.getParam("/u_lower", tmp_vector))
    {
        ROS_WARN("Warning: unable to load lower constraint for u_traj");
    }
    else
    {
        for (int i = 0; i < Constants::num_controls_u; ++i)
        {
            Constants::u_lower(i) = tmp_vector[i];
        }
    }

    // dv
    // upper
    if(!nh.getParam("/v_upper", tmp_vector))
    {
        ROS_WARN("Warning: unable to load upper constraint for v_traj");
    }
    else
    {
        for (int i = 0; i < Constants::num_controls_v; ++i)
        {
            Constants::v_upper(i) = tmp_vector[i];
        }
    }
    //lower
    if(!nh.getParam("/v_lower", tmp_vector))
    {
        ROS_WARN("Warning: unable to load lower constraint for v_traj");
    }
    else
    {
        for (int i = 0; i < Constants::num_controls_u; ++i)
        {
            Constants::v_lower(i) = tmp_vector[i];
        }
    }


    // Control initial conditions
    // du
    if(!nh.getParam("/u_hover", tmp_vector))
    {
        ROS_WARN("Warning: unable to load hover condition for u_traj");
    }
    else
    {
        for (int i = 0; i < Constants::num_controls_u; ++i)
        {
            Constants::u_hover(i) = tmp_vector[i];
        }
    }

    // dv
    if(!nh.getParam("/v_hover", tmp_vector))
    {
        ROS_WARN("Warning: unable to load hover condition for v_traj");
    }
    else
    {
        for (int i = 0; i < Constants::num_controls_v; ++i)
        {
            Constants::v_hover(i) = tmp_vector[i];
        }
    }


    // TODO: this is only used in PursuitCost. Eventually extend it to the SingleQuadrotorCost as well
    // Cost function parameters

    // Ru
    Constants::Ru = Eigen::MatrixXd::Zero(Constants::num_controls_u, Constants::num_controls_u);
    if(!nh.getParam("/Ru", tmp_vector))
    {
        ROS_WARN("Warning: unable to load Ru");
    }
    else
    {
        for(int i = 0; i < Constants::num_controls_u; ++i)
        {
            Constants::Ru(i, i) = tmp_vector[i];
        }
    }

    // Rv
    Constants::Rv = Eigen::MatrixXd::Zero(Constants::num_controls_v, Constants::num_controls_v);
    if(!nh.getParam("/Rv", tmp_vector))
    {
        ROS_WARN("Warning: unable to load Ru");
    }
    else
    {
        for(int i = 0; i < Constants::num_controls_v; ++i)
        {
            Constants::Rv(i, i) = tmp_vector[i];
        }
    }

    // Q
    Constants::Q = Eigen::MatrixXd::Zero(12,12);
    if(!nh.getParam("/Q", tmp_vector))
    {
        ROS_WARN("Warning: unable to load Ru");
    }
    else
    {
        for(int i = 0; i < 12; ++i)
        {
            Constants::Q(i, i) = tmp_vector[i];
        }
    }

    Constants::Qx_multiplier = nh.param<double>("/Qx_multiplier", Constants::Qx_multiplier);

    // Pursuit constraint
    Constants::pursuit_constrained = nh.param<bool>("/constrained", Constants::pursuit_constrained);

    // How many timesteps to read at a time from offline trajectory files
    Constants::offline_traj_batch_size = nh.param<int>("/offline_batch_size", Constants::offline_traj_batch_size);
}
