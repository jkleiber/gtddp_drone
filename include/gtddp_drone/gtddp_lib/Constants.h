#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>

namespace Constants {

    extern const double pi;
    extern const double grav;

    extern const int num_states;
    extern const int num_controls_u;
	extern const int num_controls_v;
    extern const int num_time_steps;
    extern const int num_iterations;
    extern const int num_long_legs;
    extern const int short_iterations;

    extern const double dt;
    extern const double learning_rate;

    extern const double m;
    extern const double M;
    extern const double length;
    
    extern const double Ixx;
    extern const double Iyy;
    extern const double Izz;

}

#endif // CONSTANTS_H
