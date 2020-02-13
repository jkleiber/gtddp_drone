#include "gtddp_drone/gtddp_lib/cost_functions/PursuitCost.h"

PursuitCost::PursuitCost() {
    initialize_cost_matrix();
}
PursuitCost::~PursuitCost() {}


void PursuitCost::initialize_cost_matrix()
{
    Ru = Eigen::MatrixXd::Zero(Constants::num_controls_u, Constants::num_controls_u);
    Rv = Eigen::MatrixXd::Zero(Constants::num_controls_v, Constants::num_controls_v);
    Q_f = Eigen::MatrixXd::Zero(Constants::num_states, Constants::num_states);
    Q_x = Eigen::MatrixXd::Zero(Constants::num_states / 2, Constants::num_states / 2);

    // Evader coordiantes
    Q_x(0,0) = 10; //x
    Q_x(1,1) = 10; //y
    Q_x(2,2) = 10; //z
}

double PursuitCost::calculate_cost_mm(const std::vector<Eigen::VectorXd>& x_traj, const std::vector<Eigen::VectorXd>& u_traj, const std::vector<Eigen::VectorXd>& v_traj)
{
	double total_cost = 0;
    Eigen::VectorXd pursuer(Constants::num_states / 2), evader(Constants::num_states / 2);

	for (int i = 0; i < Constants::num_time_steps - 1; i++) {
        pursuer = x_traj[i].head(Constants::num_states / 2);
        evader = x_traj[i].tail(Constants::num_states / 2);
		total_cost += 0.5 * ((evader - pursuer).transpose() * Q_x * (evader - pursuer))(0, 0) * Constants::dt;
	}

	return total_cost;
}
