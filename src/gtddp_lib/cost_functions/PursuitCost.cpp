#include "gtddp_drone/gtddp_lib/cost_functions/PursuitCost.h"

PursuitCost::PursuitCost() {
    initialize_cost_matrix();
}
PursuitCost::~PursuitCost() {}


void PursuitCost::initialize_cost_matrix()
{
    Ru = 0.5 * Eigen::MatrixXd::Identity(Constants::num_controls_u, Constants::num_controls_u);
    Rv = 1.5 * Eigen::MatrixXd::Identity(Constants::num_controls_v, Constants::num_controls_v);
    Q_f = Eigen::MatrixXd::Zero(Constants::num_states, Constants::num_states);
    Q_x = Eigen::MatrixXd::Zero(Constants::num_states, Constants::num_states);

    // Evader coordiantes
    Q_f(0,0) = 100000; //x
    Q_f(1,1) = 100000; //y
    Q_f(2,2) = 100000; //z

    Q_x = 0.0001 * Q_f;
}

double PursuitCost::calculate_cost_mm(const std::vector<Eigen::VectorXd>& x_traj, const std::vector<Eigen::VectorXd>& u_traj, const std::vector<Eigen::VectorXd>& v_traj)
{
	double total_cost = 0;
    Eigen::VectorXd pursuer(Constants::num_states), evader(Constants::num_states);

	for (int i = 0; i < Constants::num_time_steps - 1; i++) {
        // Running State Cost
        pursuer = x_traj[i];
        evader << x_traj[i].tail(Constants::num_states / 2), x_traj[i].head(Constants::num_states / 2);
		total_cost += 0.5 * ((pursuer - evader).transpose() * Q_x * (pursuer - evader))(0, 0) * Constants::dt;

        // Running control cost for pursuer
        total_cost += 0.5 * (u_traj[i].transpose() * Ru * u_traj[i])(0, 0) * Constants::dt;

        // Running control cost for evader
        total_cost += - 0.5 * (v_traj[i].transpose() * Rv * v_traj[i])(0, 0) * Constants::dt;
	}

    // Terminal Cost
    pursuer = x_traj[Constants::num_time_steps - 1];
    evader << x_traj[Constants::num_time_steps - 1].tail(Constants::num_states / 2), x_traj[Constants::num_time_steps - 1].head(Constants::num_states / 2);
	total_cost += ((pursuer - evader).transpose() * Q_f * (pursuer - evader))(0, 0);

	return total_cost;
}
