#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>

#include <ros/ros.h>
#include <string.h>

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
    extern double Ixx;
    extern double Iyy;
    extern double Izz;

    extern double du_converge_dist;
    extern double dv_converge_dist;
    extern double u0_upper, u0_lower;
    extern double u1_upper, u1_lower;
    extern double u2_upper, u2_lower;
    extern double u3_upper, u3_lower;
    extern double v0_upper, v0_lower;
    extern double v1_upper, v1_lower;
    extern double v2_upper, v2_lower;
    extern double v3_upper, v3_lower;

    extern std::string ddp_selector;

}

class ConstantLoader {
    public:
        ConstantLoader();
        ConstantLoader(ros::NodeHandle nh);

};

#endif // CONSTANTS_H
