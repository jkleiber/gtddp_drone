#ifndef CartPole_H
#define CartPole_H

#include <eigen3/Eigen/Dense>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/algebra/vector_space_algebra.hpp>

#include "../Constants.h"
#include "../Drone.h"

class CartPole : public Drone
{
    public:
        CartPole(){}
        void forward_propagate_mm(std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&);
        void linearize_dynamics_mm(const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&,
                               std::vector<Eigen::MatrixXd>&, std::vector<Eigen::MatrixXd>&, std::vector<Eigen::MatrixXd>&);
        void feedforward_controls(Eigen::VectorXd current_state, DroneTrajectory& drone_traj);

    private:
        boost::numeric::odeint::runge_kutta_dopri5<Eigen::VectorXd, double, Eigen::VectorXd, double, boost::numeric::odeint::vector_space_algebra> stepper;
        void dynamics_mm(const Eigen::VectorXd& x , Eigen::VectorXd& dxdt,  double );

        Eigen::VectorXd u_, v_;


};

#endif // PENDULUM_H
