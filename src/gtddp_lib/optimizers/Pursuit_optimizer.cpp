#include "gtddp_drone/gtddp_lib/optimizers/Pursuit_optimizer.h"

using namespace std;
using namespace Eigen;
using namespace boost::numeric::odeint;
using namespace Constants;

Pursuit_optimizer::Pursuit_optimizer() {}

Pursuit_optimizer::Pursuit_optimizer(Cost_Function* c)
{
    Ru = c->get_control_cost_u();
    Rv = c->get_control_cost_v();
    Q_x = c->get_state_cost();
    Q_f = c->get_final_cost();

//  ***** DO NOT EDIT *****
//  Initialize running cost and derivatives
    L_0_.resize(num_time_steps);
    L_x_.resize(num_time_steps);
    L_u_.resize(num_time_steps);
    L_v_.resize(num_time_steps);
    L_xx_.resize(num_time_steps);
    L_uu_.resize(num_time_steps);
    L_vv_.resize(num_time_steps);
    L_ux_.resize(num_time_steps);
    L_vx_.resize(num_time_steps);
    L_uv_.resize(num_time_steps);

//  ***** DO NOT EDIT *****
//  Initialize feedforward and feedback matrices of u and v
    lu_.resize(num_time_steps);
    lv_.resize(num_time_steps);
    Ku_.resize(num_time_steps);
    Kv_.resize(num_time_steps);

    // du
    Qu_.resize(num_time_steps);
    Qux_.resize(num_time_steps);
    Quv_.resize(num_time_steps);

    // dv
    Qv_.resize(num_time_steps);
    Qvx_.resize(num_time_steps);

} // construct a Pursuit_optimizer for the system passed as a parameter

Pursuit_optimizer::~Pursuit_optimizer() {}





/**
    This function quadratizes the cost along the given state and control trajectories: x_traj,u_traj,v_traj.
    It assigns the cost and its derivatives at time step i (t = dt * i) to the values passed in parameters 4-13
    in index i.

    @param x_traj - a vector of Eigen::VectorXd's corresponding to the state trajectory
    @param u_traj - a vector of Eigen::VectorXd's corresponding to the control_u trajectory
    @param v_traj - a vector of Eigen::VectorXd's corresponding to the control_v trajectory

*/
void Pursuit_optimizer::quadratize_cost_mm(const vector<VectorXd>& x_traj, const vector<VectorXd>& u_traj, const vector<VectorXd>& v_traj)
{
    Eigen::VectorXd x_target(num_states);
    x_target << x_traj[num_time_steps - 1].tail(num_states / 2), x_traj[num_time_steps - 1].head(num_states / 2);

    for (int i = 0; i < num_time_steps-1; i++) {
        L_0_[i]= (0.5 * u_traj[i].transpose() * Ru * u_traj[i]
                - 0.5 * v_traj[i].transpose() * Rv * v_traj[i]
                + 0.5 * (x_traj[i] - x_target).transpose() * Q_x * (x_traj[i] - x_target))(0,0);
        L_x_[i]= Q_x * (x_traj[i] - x_target); //VectorXd::Zero(num_states);//
		L_u_[i]= Ru * u_traj[i];
		L_v_[i]=-Rv * v_traj[i];
		L_xx_[i]= Q_x; // MatrixXd::Zero(num_states, num_states);//
		L_uu_[i]= Ru;
		L_vv_[i]= - Rv;
		L_ux_[i]= MatrixXd::Zero(num_controls_u, num_states);
		L_vx_[i]= MatrixXd::Zero(num_controls_v, num_states);
		L_uv_[i]= MatrixXd::Zero(num_controls_u, num_controls_v);
    }
}





/** This function takes in x_traj and Linearized dynamics A,B,C to back propagate the value ode "value_dynamics_mm" */
void Pursuit_optimizer::backpropagate_mm_rk(const vector<VectorXd>& x_traj,
                                                const vector<MatrixXd>& A, const vector<MatrixXd>& B, const vector<MatrixXd>& C)
{
    // initial values for ode
    int end = num_time_steps - 1;
    VectorXd x_target(num_states);
    x_target << x_traj[end].tail(num_states / 2), x_traj[end].head(num_states / 2);
    MatrixXd V_xx_end = Q_f;
    VectorXd V_x_end = Q_f * (x_traj[end] - x_target);
    double V_end   = 0.5 * ((x_traj[end] - x_target).transpose() * Q_f * (x_traj[end] - x_target))(0,0);

    //packaging into V_pkg, with final condition V[end]
    VectorXd V_pkg(1+num_states+num_states*num_states);
    V_pkg(0)=V_end;
    V_pkg.segment(1,num_states) =V_x_end;
    Map<RowVectorXd> v_xx_vec(V_xx_end.data(), V_xx_end.size());//flattening
    V_pkg.tail(num_states*num_states) = v_xx_vec; // column-wise stacking

    //Eigen::VectorXd dV_pkg to std::vector<double> dV_std
    std::vector<double> V_std;
    V_std.resize(V_pkg.size());
    VectorXd::Map(&V_std[0], V_pkg.size()) = V_pkg;

    double t=(num_time_steps-1)*dt;
    //main integration loop
    for (int i = end-1; i>-1 ; i=i-1) {
        // Updating the cost and linearized Dynamics at the working horizon
        L_0i_=L_0_[i];
        L_xi_=L_x_[i];
        L_ui_=L_u_[i];
        L_vi_=L_v_[i];
        L_xxi_=L_xx_[i];
        L_uui_=L_uu_[i];
        L_vvi_=L_vv_[i];
        L_uxi_=L_ux_[i];
        L_vxi_=L_vx_[i];
        L_uvi_=L_uv_[i];

        Ai_=A[i];
        Bi_=B[i];
        Ci_=C[i];

        //numerically solve the value ode
        integrate_adaptive(controlled_stepper, std::bind(&::Pursuit_optimizer::value_dynamics_mm, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), V_std , t , t-dt , -dt );
        t+=-dt;

        //save the obtained gains to the private members
        //caveat! these are backward propagated ques (i: num_time_steps -> 0)
        lu_[i]=lui_;
        lv_[i]=lvi_;
        Ku_[i]=Kui_;
        Kv_[i]=Kvi_;

        // du
        Qux_[i] = Qux;
        Qu_[i] = Qu;
        Quv_[i] = Quv;

        // dv
        Qv_[i] = Qv;
        Qvx_[i] = Qvx;

    } //back propagating loop
}//back_propagate();





/**
 This function updates the controls using the optimal control corrections and the learning rate
 according to u_new = u_old + learning_rate * du_star. The updated control trajectory is written
 into the parameter 5, u_traj.
 */
void Pursuit_optimizer::update_controls_mm(const vector<VectorXd>& dx_traj,
                                          vector<VectorXd>& u_traj, vector<VectorXd>& v_traj)
{
    // Vectors for delta u and delta v
    VectorXd du(num_controls_u);
    VectorXd dv(num_controls_v);

    // Control Constraint DDP
    Program qp(CGAL::SMALLER);
    Solution sol;

    // Quadratic program parameters
    Eigen::VectorXd b(2*num_controls_u), c;                           // b and c vectors
    Eigen::VectorXd b1(num_controls_u), b2(num_controls_u);           // intermediate b vectors
    Eigen::VectorXd lower_u(num_controls_u), upper_u(num_controls_u); // upper and lower bounds for u
    Eigen::VectorXd lower_v(num_controls_v), upper_v(num_controls_v); // upper and lower bounds for v

    // Solver convergence
    bool first_run;
    Eigen::VectorXd last_du(num_controls_u), last_dv(num_controls_v);
    double dist_u = 0.0, dist_v = 0.0;

    // Set c0 to 0
    qp.set_c0(0);

    // Establish boundaries
    // du
    upper_u << u0_upper, u1_upper, u2_upper, u3_upper;
    lower_u << u0_lower, u1_lower, u2_lower, u3_lower;

    // dv
    upper_v << v0_upper, v1_upper, v2_upper, v3_upper;
    lower_v << v0_lower, v1_lower, v2_lower, v3_lower;


    // Update the controls using a QP solver
    for (int i = 0; i < num_time_steps-1; i++) {
        // Find dv
        dv = lv_[i] + Kv_[i] * dx_traj[i];
        //du = lu_[i] + Ku_[i] * dx_traj[i];
        first_run = true;

        /**
         * Double Control constraint DDP logic
         */

        while(first_run || dist_u >= Constants::du_converge_dist || dist_v >= Constants::dv_converge_dist)
        {
            //////////////////
            // du
            /////////////////
            // Calculate boundaries for du
            b1 = lower_u - u_traj[i];
            b2 = upper_u - u_traj[i];

            // Set control boundaries
            for(int i = 0; i < num_controls_u; ++i)
            {
                qp.set_l(i, true, b1(i));
                qp.set_u(i, true, b2(i));
            }

            // Calculate linear objective function for du
            c = Qux_[i]*dx_traj[i] + Quv_[i]*dv + Qu_[i];

            // Add quadratic and linear objective functions to the program solver
            // Also add boundaries to the solver
            for (int i = 0; i < Constants::num_controls_u; ++i)
            {
                qp.set_c(i, c(i));
                for (int j = 0; j < Constants::num_controls_u; ++j)
                {
                    qp.set_d(i, j, Quu(i, j));
                }
            }

            // solve quadratic program to find du
            sol = CGAL::solve_quadratic_program(qp, ET());

            // Find du after performing the quadratic programming step if the solution is feasible
            if(sol.solves_quadratic_program(qp) && !sol.is_infeasible())
            {
                Solution::Variable_value_iterator it = sol.variable_values_begin();
                Solution::Variable_value_iterator end = sol.variable_values_end();

                for (int u = 0; it != end; ++it, ++u) {
                    du(u) = CGAL::to_double(*it);
                }
            }
            // QP failed, so use normal du
            else
            {
                du = lu_[i] + Ku_[i] * dx_traj[i];
            }

            //////////////////
            // dv
            /////////////////
            // Calculate boundaries for dv
            b1 = lower_v - v_traj[i];
            b2 = upper_v - v_traj[i];

            // Set control boundaries
            for(int i = 0; i < num_controls_v; ++i)
            {
                qp.set_l(i, true, b1(i));
                qp.set_u(i, true, b2(i));
            }

            // Calculate linear objective function for dv
            c = -(Qvx_[i]*dx_traj[i] + Quv_[i].transpose()*du + Qv_[i]);

            // Add quadratic and linear objective functions to the program solver
            // Also add boundaries to the solver
            for (int i = 0; i < Constants::num_controls_v; ++i)
            {
                qp.set_c(i, c(i));
                for (int j = 0; j < Constants::num_controls_v; ++j)
                {
                    qp.set_d(i, j, -Qvv(i, j));
                }
            }

            // solve quadratic program to find dv
            sol = CGAL::solve_quadratic_program(qp, ET());

            // Find du after performing the quadratic programming step if the solution is feasible
            if(sol.solves_quadratic_program(qp) && !sol.is_infeasible())
            {
                Solution::Variable_value_iterator it = sol.variable_values_begin();
                Solution::Variable_value_iterator end = sol.variable_values_end();

                for (int v = 0; it != end; ++it, ++v) {
                    dv(v) = CGAL::to_double(*it);
                }
            }
            // QP failed, so use normal dv
            else
            {
                std::cout << "dv ERROR\n";
                dv = lv_[i] + Kv_[i] * dx_traj[i];
            }

            // Calculate if the quadratic program needs to continue
            // If it is the first run, continue by setting the distances above the thresholds
            if (first_run)
            {
                first_run = false;
                dist_u = du_converge_dist + 1.0;
                dist_v = dv_converge_dist + 1.0;
                last_du = du;
                last_dv = dv;
            }
            // If this is not the first run, calculate the distances between solutions
            // If these distances are both small enough, then the answer will be accepted
            else
            {
                dist_u = (du - last_du).dot(du - last_du);
                dist_v = (dv - last_dv).dot(dv - last_dv);
                last_du = du;
                last_dv = dv;
            }

            //std::cout << du << std::endl << dv << std::endl << dist_u << ", " << dist_v << std::endl;

            //std::cout << du << "\t" << dv << std::endl;
        }

        // Update controls
        u_traj[i] = u_traj[i] + learning_rate * du;
        v_traj[i] = v_traj[i] + learning_rate * dv;
    }
}

/**
 * @brief Initializes the controls to hover for pursuit mode
 *
 * @param x_traj
 * @param u_traj
 * @param v_traj
 * @param dx_traj
 */
void Pursuit_optimizer::initialize_trajectories(vector<VectorXd>& x_traj, vector<VectorXd>& u_traj, vector<VectorXd>& v_traj, vector<VectorXd>& dx_traj)
{
    for (int i = 0; i < num_time_steps; i++) {
        x_traj[i] = VectorXd::Zero(num_states);
    }

    for (int i = 0; i < num_time_steps-1; i++) {
        u_traj[i] = VectorXd::Zero(num_controls_u);
    }

	for (int i = 0; i < num_time_steps - 1; i++) {
		v_traj[i] = VectorXd::Zero(num_controls_v);
        v_traj[i](0) = 6;
	}

    for (int i = 0; i < num_time_steps-1; i++) {
        dx_traj[i] = VectorXd::Zero(num_states);
    }
}
