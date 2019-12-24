#include "gtddp_drone/gtddp_lib/DDP_Optimizer.h"

using namespace std;
using namespace Eigen;
using namespace boost::numeric::odeint;
using namespace Constants;


/**
    This function quadratizes the cost along the given state and control trajectories: x_traj,u_traj,v_traj.
    It assigns the cost and its derivatives at time step i (t = dt * i) to the values passed in parameters 4-13
    in index i.

    @param x_traj - a vector of Eigen::VectorXd's corresponding to the state trajectory
    @param u_traj - a vector of Eigen::VectorXd's corresponding to the control_u trajectory
    @param v_traj - a vector of Eigen::VectorXd's corresponding to the control_v trajectory

*/
void DDP_Optimizer::quadratize_cost_mm(const vector<VectorXd>& x_traj, const vector<VectorXd>& u_traj, const vector<VectorXd>& v_traj)
{
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





/** This function is the backward ode function of Values paramatized with the running cost over horizon */
void DDP_Optimizer::value_dynamics_mm(const std::vector<double>& V_std , std::vector<double>& dV_std,  double t)
{
    //std::vector<double> V_std to Eigen::VectorXd V_pkg
    std::vector<double> sV_std = V_std;
    double* ptr = &sV_std[0];
    Eigen::Map<Eigen::VectorXd> V_pkg(ptr, sV_std.size());

    // double   V =V_pkg(0); not used
    VectorXd V_x=V_pkg.segment(1,num_states);
    MatrixXd V_xx(num_states, num_states);
    //column-wise unpackaging
    for(int j=0;j<num_states;j++) {
        V_xx.col(j)=V_pkg.segment(num_states*(j+1)+1,num_states);
    }
    //
    Qx= Ai_.transpose() * V_x+ L_xi_;              //(num_states);
    Qu= Bi_.transpose() * V_x + L_ui_;             //(num_controls_u);
    Qv= Ci_.transpose() * V_x + L_vi_;             //(num_controls_v);
    Qxx= L_xxi_ + 2 * V_xx * Ai_;                  //(num_states, num_states);
    Quu= L_uui_;                                   //(num_controls_u, num_controls_u);
    Qvv= L_vvi_;                                   //(num_controls_v, num_controls_v);
    Qux= L_uxi_ + Bi_.transpose() * V_xx;          //(num_controls_u, num_states);
    Qvx = L_vxi_ + Ci_.transpose() * V_xx;         //(num_controls_v, num_states);
    Quv= L_uvi_;                                   //(num_controls_u, num_controls_v);
    Qvu= Quv.transpose();                          //(num_controls_v, num_controls_u);

    Quu_inv= Quu.inverse();                        //(num_controls_u, num_controls_u);
    Qvv_inv= Qvv.inverse();                        //(num_controls_v, num_controls_v);
    G= Quu - Quv * Qvv_inv * Qvu;                  //(num_controls_u, num_controls_u);
    H= Qvv - Qvu * Quu_inv * Quv;                  //(num_controls_v, num_controls_v);
    G_inv= G.inverse();                            //(num_controls_u, num_controls_u);
    H_inv= H.inverse();                            //(num_controls_v, num_controls_v);

    lui_ = -G_inv * (Qu - Quv * Qvv_inv * Qv);
    Kui_ = -G_inv * (Qux - Quv * Qvv_inv * Qvx);
    lvi_ = -H_inv * (Qv - Qvu * Quu_inv * Qu);
    Kvi_ = -H_inv * (Qvx - Qvu * Quu_inv * Qux);

    MatrixXd dVxxdt =-(Qxx + Kui_.transpose()*Quu*Kui_ + Kvi_.transpose()*Qvv*Kvi_ + 2 * Kui_.transpose()*Qux + 2 * Kvi_.transpose()*Qvx + 2 * Kui_.transpose()*Quv*Kvi_);
    VectorXd dVxdt = -(Qx + Kui_.transpose()*Qu + Kvi_.transpose()*Qv + Qux.transpose()*lui_ + Qvx.transpose()*lvi_ + Kui_.transpose() *Quu*lui_ + Kvi_.transpose() *Qvv*lvi_ + Kui_.transpose() *Quv*lvi_ + Kvi_.transpose() *Qvu*lui_);
    double dVdt =-(L_0i_ + lui_.transpose()*Qu + lvi_.transpose()*Qv + 0.5*lui_.transpose()*Quu*lui_ + lui_.transpose()*Quv*lvi_+ 0.5*lvi_.transpose()*Qvv*lvi_);
    MatrixXd sdVxxdt = 0.5*(dVxxdt + dVxxdt.transpose()); //imposing symmetry of dVxxdt

    //packaging into Eigen::vec dV_pkg
    VectorXd dV_pkg;
    dV_pkg.setZero(1+num_states+num_states*num_states);
    dV_pkg(0)= dVdt;
    dV_pkg.segment(1,num_states)= dVxdt;
    Map<RowVectorXd> dv_xx_vec(sdVxxdt.data(), sdVxxdt.size());
    dV_pkg.tail(num_states*num_states)= dv_xx_vec;

    // type casting back to std::vec (Eigen -> std::vec)
    dV_std.resize(dV_pkg.size());
    VectorXd::Map(&dV_std[0], dV_pkg.size()) = dV_pkg;

};




/** This function is the ode function for updating dx with given control gains lu, lv, Ku, Kv*/
struct dx_update_mm
{
    MatrixXd A;
    MatrixXd B;
    MatrixXd C;

    VectorXd lu;
    VectorXd lv;
    MatrixXd Ku;
    MatrixXd Kv;
    dx_update_mm(MatrixXd Ai,MatrixXd Bi,MatrixXd Ci, VectorXd lui, VectorXd lvi, MatrixXd Kui, MatrixXd Kvi):
                A(Ai),B(Bi),C(Ci),lu(lui),lv(lvi),Ku(Kui),Kv(Kvi){}

    void operator()(const Eigen::VectorXd& dx , Eigen::VectorXd& dx_updated,  double t)
    {
        dx_updated =A*dx+B*(lu+Ku*dx)+C*(lv+Kv*dx);
    }
};

/** forward propagating in order to update dx with given A, B, C, lu, lv, Ku, Kv */
void DDP_Optimizer::forward_propagate_mm_rk(vector<VectorXd>& dx_traj,
                                           const vector<MatrixXd>& A, const vector<MatrixXd>& B, const vector<MatrixXd>& C)
{
    //initial condition
    VectorXd dx=dx_traj[0];
    double t=0;
    //main integration loop
    for (int i = 0; i < (num_time_steps - 1); i++)
    {
        //update dx
        dx_update_mm dum(A[i], B[i] ,C[i], lu_[i],lv_[i], Ku_[i], Kv_[i]);
        stepper.do_step(dum, dx, t, dt);
        dx_traj[i+1]=dx;
        t+=-dt;
    } //stepper loop

} //forward_propagate_mm_rk


/**
    This function initializes the state and control trajectories (the parameters) to
    vectors of zero VectorXd's
*/
void DDP_Optimizer::initialize_trajectories_to_zero_mm(vector<VectorXd>& x_traj, vector<VectorXd>& u_traj, vector<VectorXd>& v_traj, vector<VectorXd>& dx_traj)
{
    for (int i = 0; i < num_time_steps; i++) {
        x_traj[i] = VectorXd::Zero(num_states);
    }

    for (int i = 0; i < num_time_steps-1; i++) {
        u_traj[i] = VectorXd::Zero(num_controls_u);
    }

	for (int i = 0; i < num_time_steps - 1; i++) {
		v_traj[i] = VectorXd::Zero(num_controls_v);
	}

    for (int i = 0; i < num_time_steps-1; i++) {
        dx_traj[i] = VectorXd::Zero(num_states);
    }
}
