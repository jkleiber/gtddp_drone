#include "gtddp_drone/gtddp_lib/cost_functions/SingleQuadrotorCost.h"

using namespace std;
using namespace Eigen;
using namespace Constants;

SingleQuadrotorCost::SingleQuadrotorCost() {}
SingleQuadrotorCost::SingleQuadrotorCost(Eigen::VectorXd x_t)
{
    initialize_cost_matrix();
    x_target = x_t;
}
SingleQuadrotorCost::~SingleQuadrotorCost() {}

void SingleQuadrotorCost::initialize_cost_matrix(){

    /*  BEGIN COST MATRIX INITIALIZATION
     Ru is num_controls_u x num_controls_u matrix of control cost weights for u
     Rv is num_controls_v x num_controls_v matrix of control cost weights for v
     Q_x is num_states x num_states matrix of state dependent cost weights
     Q_f is num_states x num_states matrix of final state dependent cost weights

     Edit the values in these matrices as necessary.
     */
    /**
     * Intuition for Ru and Rv: Higher values on the diagonal result in more restrictive control. Ru < Rv to ensure convergence
     *
     * Intuition for Q_f: Higher values on diagonal results in more emphasis on this performance metric
     *                    Q_f is the cost matrix, so to make certain errors more costly, add value to the appropriate state variable
     *
     * Intuition for Q_x: Scale Q_f by a larger amount to get to the target faster, smaller Q_x is more stable though
     */

    // Set the costs from the paramters file
    Ru = Constants::Ru;
    Rv = Constants::Rv;
    Q_f = Constants::Q;

    Q_x = Constants::Qx_multiplier*Q_f;

}

double SingleQuadrotorCost::calculate_cost_mm(const vector<VectorXd>& x_traj, const vector<VectorXd>& u_traj, const vector<VectorXd>& v_traj)
{
	double total_cost = 0;

	for (int i = 0; i < num_time_steps - 1; i++) {
		total_cost += 0.5 * (u_traj[i].transpose() * Ru * u_traj[i])(0, 0) * dt;
		total_cost += - 0.5 * (v_traj[i].transpose() * Rv * v_traj[i])(0, 0) * dt;
		total_cost += 0.5 * ((x_traj[i] - x_target).transpose() * Q_x * (x_traj[i] - x_target))(0, 0) * dt;
	}

	total_cost += ((x_traj[num_time_steps - 1] - x_target).transpose() * Q_f * (x_traj[num_time_steps - 1] - x_target))(0, 0);

	return total_cost;
}
