#include "gtddp_drone/gtddp_lib/cost_functions/PursuitCost.h"

PursuitCost::PursuitCost() {
    initialize_cost_matrix();
}
PursuitCost::~PursuitCost() {}


void PursuitCost::initialize_cost_matrix()
{
    Ru = Constants::Ru;
    Rv = Constants::Rv;
    Q_f = Eigen::MatrixXd::Zero(Constants::num_states, Constants::num_states);
    Q_x = Eigen::MatrixXd::Zero(Constants::num_states, Constants::num_states);

    // Intermediate Q and L matrices for solving for Q_f and Q_x
    Eigen::MatrixXd Q = Constants::Q;
    Eigen::MatrixXd L = Eigen::MatrixXd::Zero(Constants::num_states / 2, Constants::num_states);
    L << (-1) * Eigen::MatrixXd::Identity(Constants::num_states / 2, Constants::num_states / 2), Eigen::MatrixXd::Identity(Constants::num_states / 2, Constants::num_states / 2);

    // Translate to Q_x
    Q_f = (L.transpose() * Q * L);

    Q_x = Constants::Qx_multiplier * Q_f;
}

double PursuitCost::calculate_cost_mm(const std::vector<Eigen::VectorXd>& x_traj, const std::vector<Eigen::VectorXd>& u_traj, const std::vector<Eigen::VectorXd>& v_traj)
{
	double total_cost = 0;

	for (int i = 0; i < Constants::num_time_steps - 1; i++) {
        // Running State Cost
		total_cost += 0.5 * (x_traj[i].transpose() * Q_x * x_traj[i])(0, 0) * Constants::dt;

        // Running control cost for pursuer
        total_cost += 0.5 * ((u_traj[i] - Constants::u_hover).transpose() * Ru * (u_traj[i] - Constants::u_hover))(0, 0) * Constants::dt;

        // Running control cost for evader
        total_cost += - 0.5 * ((v_traj[i] - Constants::v_hover).transpose() * Rv * (v_traj[i] - Constants::v_hover))(0, 0) * Constants::dt;
	}

    // Terminal Cost
	total_cost += (x_traj[Constants::num_time_steps - 1].transpose() * Q_f * x_traj[Constants::num_time_steps - 1])(0, 0);

	return total_cost;
}
