#include "gtddp_drone/gtddp_lib/Cost_Function.h"

using namespace std;
using namespace Eigen;
using namespace Constants;

Cost_Function::Cost_Function() {}
Cost_Function::Cost_Function(Eigen::VectorXd x_t)
{
    initialize_cost_matrix();
    x_target = x_t;
}
Cost_Function::~Cost_Function() {}

void Cost_Function::initialize_cost_matrix(){

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
    Ru = MatrixXd::Identity(num_controls_u, num_controls_u); //0.015 *
    Rv = 2.5 * MatrixXd::Identity(num_controls_v, num_controls_v);
    Q_x = MatrixXd::Identity(num_states, num_states);
    Q_f = MatrixXd::Identity(num_states, num_states);

    //Set Ru values individually
    Ru(0, 0) = 0.4;  //thrust
    Ru(1, 1) = 0.5;   //u1 moment //0.04 worked pretty well in case this breaks
    Ru(2, 2) = 0.5;   //u2 moment
    Ru(3, 3) = 0.5;   //u3 moment

    //Quadrotor Cost
    Q_f(0,0) = 1000000; //x
    Q_f(1,1) = 1000000; //y
    Q_f(2,2) = 1000000; //z
    //velocity states
    Q_f(3,3) = 10000;   //x dot
    Q_f(4,4) = 10000;   //y dot
    Q_f(5,5) = 10000;   //z dot
    //angular states
    Q_f(6,6) = 10000;  //roll
    Q_f(7,7) = 10000;  //pitch
    Q_f(8,8) = 10000;  //yaw
    //angular velocity states
    Q_f(9,9) = 1000;    //roll rate
    Q_f(10,10) = 1000;  //pitch rate
    Q_f(11,11) = 10000; //yaw rate

    Q_x = 0.0001*Q_f;

}

double Cost_Function::calculate_cost_mm(const vector<VectorXd>& x_traj, const vector<VectorXd>& u_traj, const vector<VectorXd>& v_traj)
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
