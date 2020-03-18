#include "gtddp_drone/gtddp_lib/cost_functions/PursuitCost.h"

PursuitCost::PursuitCost() {
    initialize_cost_matrix();
}
PursuitCost::~PursuitCost() {}


void PursuitCost::initialize_cost_matrix()
{
    Ru = 0.08 * Eigen::MatrixXd::Identity(Constants::num_controls_u, Constants::num_controls_u);
    Rv = 2.0 * Eigen::MatrixXd::Identity(Constants::num_controls_v, Constants::num_controls_v);
    Q_f = Eigen::MatrixXd::Zero(Constants::num_states, Constants::num_states);
    Q_x = Eigen::MatrixXd::Zero(Constants::num_states, Constants::num_states);

    // Intermediate Q and L matrices for solving for Q_f and Q_x
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(Constants::num_states / 2, Constants::num_states / 2);
    Eigen::MatrixXd L = Eigen::MatrixXd::Zero(Constants::num_states / 2, Constants::num_states);
    L << (-1) * Eigen::MatrixXd::Identity(Constants::num_states / 2, Constants::num_states / 2), Eigen::MatrixXd::Identity(Constants::num_states / 2, Constants::num_states / 2);

    // Positional cost
    Q(0,0) = 1000; //x
    Q(1,1) = 1000; //y
    Q(2,2) = 1000; //z
    /*
    // Velocity cost
    Q(3,3) = 1000; //x dot
    Q(4,4) = 1000; //y dot
    Q(5,5) = 1000; //z dot
    */

    // Translate to Q_f
    Q_f = (L.transpose() * Q * L);

    //Q_x = 0.00001 * Q_f;
}

double PursuitCost::calculate_cost_mm(const std::vector<Eigen::VectorXd>& x_traj, const std::vector<Eigen::VectorXd>& u_traj, const std::vector<Eigen::VectorXd>& v_traj)
{
	double total_cost = 0;

	for (int i = 0; i < Constants::num_time_steps - 1; i++) {
        // Running State Cost
		total_cost += 0.5 * (x_traj[i].transpose() * Q_x * x_traj[i])(0, 0) * Constants::dt;

        // Running control cost for pursuer
        total_cost += 0.5 * (u_traj[i].transpose() * Ru * u_traj[i])(0, 0) * Constants::dt;

        // Running control cost for evader
        total_cost += - 0.5 * (v_traj[i].transpose() * Rv * v_traj[i])(0, 0) * Constants::dt;
	}

    // Terminal Cost
	total_cost += (x_traj[Constants::num_time_steps - 1].transpose() * Q_f * x_traj[Constants::num_time_steps - 1])(0, 0);

	return total_cost;
}
