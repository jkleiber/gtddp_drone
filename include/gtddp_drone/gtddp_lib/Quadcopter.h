#ifndef Quadcopter_h
#define Quadcopter_h

#include <eigen3/Eigen/Dense>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>

#include "Constants.h"

class Quadcopter
{
public:
    /**
     This constructor instantiates the Quadcopter object by making a call to the
     superclass constructor in System. Every System subclass (Cart_Pole,
     Quadcopter, etc.) must have this constructor, only changing the handle
     of the constructor.
     @param x_0_i - a num_states x 1 vector of the initial state of the System
     @param x_target_i - a num_states x 1 vector of the target state of the System
     */
    Quadcopter(){} // Call the System constructor.
    void forward_propagate_mm(std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&);
    void linearize_dynamics_mm(const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&,
                               std::vector<Eigen::MatrixXd>&, std::vector<Eigen::MatrixXd>&, std::vector<Eigen::MatrixXd>&);
    void feedforward_controls(Eigen::VectorXd current_state, const std::vector<Eigen::VectorXd>& u_traj, const std::vector<Eigen::MatrixXd>& K_traj, std::vector<Eigen::VectorXd>& x_traj);
private:
    boost::numeric::odeint::runge_kutta_dopri5<Eigen::VectorXd, double, Eigen::VectorXd, double, boost::numeric::odeint::vector_space_algebra> stepper;
    void dynamics_mm(const Eigen::VectorXd& x , Eigen::VectorXd& dxdt,  double );
    Eigen::VectorXd ui_;
    Eigen::VectorXd vi_;
    
};

#endif /* Quadcopter_h */
