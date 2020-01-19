#include "gtddp_drone/gtddp_lib/GT_DDP_optimizer.h"
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace boost::numeric::odeint;
using namespace Constants;

GT_DDP_optimizer::GT_DDP_optimizer() {}

GT_DDP_optimizer::GT_DDP_optimizer(Cost_Function c)
{
    cost = c;
    Ru = c.get_control_cost_u();
    Rv = c.get_control_cost_v();
    Q_x = c.get_state_cost();
    Q_f = c.get_final_cost();
    x_target = c.get_target_state();

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

} // construct a GT_DDP_optimizer for the system passed as a parameter

GT_DDP_optimizer::~GT_DDP_optimizer() {}




/** This function takes in x_traj and Linearized dynamics A,B,C to back propagate the value ode "value_dynamics_mm" */
void GT_DDP_optimizer::backpropagate_mm_rk(const vector<VectorXd>& x_traj,
                                                const vector<MatrixXd>& A, const vector<MatrixXd>& B, const vector<MatrixXd>& C)
{
    // initial values for ode
    int end = num_time_steps - 1;
    MatrixXd V_xx_end = Q_f;
    VectorXd V_x_end = Q_f * (x_traj[end] - x_target);
    double V_end   = 0.5 * ((x_traj[end] - x_target).transpose() * Q_f * (x_traj[end] - x_target))(0,0);

    //packaging into V_pkg, with final condition V[end]
    VectorXd V_pkg(1+num_states+num_states*num_states);
    V_pkg(0)=V_end;
    V_pkg.segment(1,num_states) =V_x_end;
    Map<RowVectorXd> v_xx_vec(V_xx_end.data(), V_xx_end.size());//flatening
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
        integrate_adaptive(controlled_stepper, std::bind(&::GT_DDP_optimizer::value_dynamics_mm, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), V_std , t , t-dt , -dt );
        t+=-dt;

        //save the obtained gains to the private members
        //caveat! these are backward propagated ques (i: num_time_steps -> 0)
        lu_[i]=lui_;
        lv_[i]=lvi_;
        Ku_[i]=Kui_;
        Kv_[i]=Kvi_;

    } //back propagating loop
}//back_propagate();







/**
 This function updates the controls using the optimal control corrections and the learning rate
 according to u_new = u_old + learning_rate * du_star. The updated control trajectory is written
 into the parameter 5, u_traj.
 */
void GT_DDP_optimizer::update_controls_mm(const vector<VectorXd>& dx_traj,
                                          vector<VectorXd>& u_traj, vector<VectorXd>& v_traj)
{
    // Vectors for delta u and delta v
    VectorXd du(num_controls_u);
    VectorXd dv(num_controls_v);

    // Update the controls using a QP solver
    for (int i = 0; i < num_time_steps-1; i++)
    {
        du = lu_[i] + Ku_[i] * dx_traj[i];
        dv = lv_[i] + Kv_[i] * dx_traj[i];

        // Update controls
        u_traj[i] = u_traj[i] + learning_rate * du;
        v_traj[i] = v_traj[i] + learning_rate * dv;
    }

}
