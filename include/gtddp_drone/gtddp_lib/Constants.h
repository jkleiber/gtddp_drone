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
    extern double M;
    extern double length;

    extern double Ixx;
    extern double Iyy;
    extern double Izz;

    extern std::string ddp_selector;

}

class ConstantLoader {
    public:
        ConstantLoader();
        ConstantLoader(ros::NodeHandle nh);

};

#endif // CONSTANTS_H
