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

    // DDP Constant ints
    // Change as necessary, do not remove
////   cart-pole system
//     extern const int num_states(4);
//     extern const int num_controls_u(1);
//	   extern const int num_controls_v(1);
//    
   //Quadrotor system
    extern const int num_states(12);
    extern const int num_controls_u(4);
    extern const int num_controls_v(4);
//    
    extern const int num_time_steps(500);
    extern const int num_iterations(3);

    // DDP Constant doubles
    // Change as necessary, do not remove
    extern const double dt(0.01);
    extern const double learning_rate(0.3);

    // System Constant doubles
    // Change, add, or remove as necessary
    extern const double m(1.52);
    extern const double M(10.0);
    extern const double length(0.5);
    extern const double Ixx(0.0347563);
    extern const double Iyy(0.0458929);
    extern const double Izz( 0.0977);

}
