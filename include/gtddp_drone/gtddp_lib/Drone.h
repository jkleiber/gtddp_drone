#ifndef DRONE_H
#define DRONE_H

#include <eigen3/Eigen/Dense>
#include <queue>

class Drone {
    public:
        virtual void forward_propagate_mm(std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&) = 0;

        virtual void linearize_dynamics_mm(const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&,
                               std::vector<Eigen::MatrixXd>&, std::vector<Eigen::MatrixXd>&, std::vector<Eigen::MatrixXd>&) = 0;

        virtual void feedforward_controls(Eigen::VectorXd current_state, const std::deque<Eigen::VectorXd>& u_traj, const std::deque<Eigen::MatrixXd>& K_traj, std::deque<Eigen::VectorXd>& x_traj) = 0;
};

#endif
