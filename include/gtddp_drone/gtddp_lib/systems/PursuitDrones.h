#ifndef PursuitDrones_h
#define PursuitDrones_h

#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>

#include "../Constants.h"
#include "../Drone.h"

class PursuitDrones : public Drone
{
public:
    PursuitDrones(){}
    void forward_propagate_mm(std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&);
    void linearize_dynamics_mm(const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&,
                               std::vector<Eigen::MatrixXd>&, std::vector<Eigen::MatrixXd>&, std::vector<Eigen::MatrixXd>&);
    void feedforward_controls(Eigen::VectorXd current_state, const std::deque<Eigen::VectorXd>& u_traj, const std::deque<Eigen::MatrixXd>& K_traj, std::deque<Eigen::VectorXd>& x_traj);

private:
    boost::numeric::odeint::runge_kutta_dopri5<Eigen::VectorXd, double, Eigen::VectorXd, double, boost::numeric::odeint::vector_space_algebra> stepper;
    void dynamics_mm(const Eigen::VectorXd& x , Eigen::VectorXd& dxdt,  double );
    Eigen::VectorXd ui_;
    Eigen::VectorXd vi_;

};

#endif /* PursuitDrones_h */
