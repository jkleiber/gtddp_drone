#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>

#include <ros/ros.h>
#include <string.h>
#include <eigen3/Eigen/Dense>

namespace Constants {

    extern const double pi;
    extern const double grav;

    extern int num_states;
    extern int num_controls_u;
	extern int num_controls_v;
    extern int num_time_steps;
    extern int num_iterations;
    extern int num_long_legs;
    extern int short_iterations;

    extern double dt;
    extern double learning_rate;

    extern double m;
    extern double length;
    extern double M;
    extern double Ixx;
    extern double Iyy;
    extern double Izz;

    extern double du_converge_dist;
    extern double dv_converge_dist;
    extern Eigen::VectorXd u_upper, u_lower, v_upper, v_lower;

    // Initial conditions
    extern Eigen::VectorXd u_hover, v_hover;

    // Cost function parameters
    extern Eigen::MatrixXd Ru;
    extern Eigen::MatrixXd Rv;
    extern Eigen::MatrixXd Q;
    extern double Qx_multiplier;

    // Pursuit constraint toggle
    extern bool pursuit_constrained;

    // Offline trajectory batch size
    extern int offline_traj_batch_size;

    extern std::string ddp_selector;
    extern std::string constraint;

    // Show debugging prints
    extern bool debug_mode;
}

class ConstantLoader {
    public:
        ConstantLoader();
        ConstantLoader(ros::NodeHandle nh);

};

#endif // CONSTANTS_H
