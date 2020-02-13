#ifndef DRONE_H
#define DRONE_H

#include <eigen3/Eigen/Dense>
#include <queue>

typedef struct drone_traj_t{
    drone_traj_t() : x_traj(0), u_traj(0), v_traj(0), Ku_traj(0), Kv_traj(0){}

    // State trajectory
    std::deque<Eigen::VectorXd> x_traj;

    // Stabilizing control trajectory
    std::deque<Eigen::VectorXd> u_traj;
    std::deque<Eigen::MatrixXd> Ku_traj;

    // Destabilizing control trajectory
    std::deque<Eigen::VectorXd> v_traj;
    std::deque<Eigen::MatrixXd> Kv_traj;
} DroneTrajectory;

class Drone {
    public:
        virtual void forward_propagate_mm(std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&) = 0;

        virtual void linearize_dynamics_mm(const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&,
                               std::vector<Eigen::MatrixXd>&, std::vector<Eigen::MatrixXd>&, std::vector<Eigen::MatrixXd>&) = 0;

        virtual void feedforward_controls(Eigen::VectorXd current_state, DroneTrajectory& drone_traj) = 0;
};

#endif
