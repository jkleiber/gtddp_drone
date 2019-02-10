#ifndef CART_POLE_H
#define CART_POLE_H

#include <eigen3/Eigen/Dense>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>

#include "Constants.h"

class Cart_Pole
{
    public:
        /**
            This constructor instantiates the Cart_Pole object by making a call to the
            superclass constructor in System. Every System subclass (Cart_Pole,
            Quadrotor, etc.) must have this constructor, only changing the handle
            of the constructor.
            @param x_0_i - a num_states x 1 vector of the initial state of the System
            @param x_target_i - a num_states x 1 vector of the target state of the System
        */
        Cart_Pole(){} // Call the System constructor.
        void forward_propagate_mm(std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&);
        void linearize_dynamics_mm(const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&,
                               std::vector<Eigen::MatrixXd>&, std::vector<Eigen::MatrixXd>&, std::vector<Eigen::MatrixXd>&);

    private:
        boost::numeric::odeint::runge_kutta_dopri5<Eigen::VectorXd, double, Eigen::VectorXd, double, boost::numeric::odeint::vector_space_algebra> stepper;


};

#endif // PENDULUM_H
