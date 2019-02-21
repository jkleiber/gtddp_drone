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
    Ru =  0.01 * MatrixXd::Identity(num_controls_u, num_controls_u);
    Rv = 0.1 * MatrixXd::Identity(num_controls_v, num_controls_v);
    Q_x = MatrixXd::Identity(num_states, num_states);
    Q_f = MatrixXd::Identity(num_states, num_states);

    //Quadrotor Cost
    Q_f(0,0) = 1000000;
    Q_f(1,1) = 1000000;
    Q_f(2,2) = 1000000;
    //    % angular states
    Q_f(3,3) = 10000;
    Q_f(4,4) = 10000;
    Q_f(5,5) = 10000;
    //    % velocity states
    Q_f(6,6) = 100000;
    Q_f(7,7) = 100000;
    Q_f(8,8) = 100000;
    //    % angular velocity states
    Q_f(9,9) = 1000;
    Q_f(10,10) = 1000;
    Q_f(11,11) = 1000;
    
}

double Cost_Function::calculate_cost_mm(const vector<VectorXd>& x_traj, const vector<VectorXd>& u_traj, const vector<VectorXd>& v_traj)
{
	double total_cost = 0;

	for (int i = 0; i < num_time_steps - 1; i++) {
		total_cost += 0.5 * (u_traj[i].transpose() * Ru * u_traj[i])(0, 0) * dt;
		total_cost += - 0.5 * (v_traj[i].transpose() * Rv * v_traj[i])(0, 0) * dt;
		//total_cost += 0.5 * ((x_traj[i] - x_target).transpose() * Q_x * (x_traj[i] - x_target))(0, 0) * dt;
	}

	total_cost += ((x_traj[num_time_steps - 1] - x_target).transpose() * Q_f * (x_traj[num_time_steps - 1] - x_target))(0, 0);

	return total_cost;
}
