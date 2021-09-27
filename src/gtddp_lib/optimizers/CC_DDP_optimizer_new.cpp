#include "gtddp_drone/gtddp_lib/optimizers/CC_DDP_optimizer_new.h"

using namespace std;
using namespace Eigen;
using namespace boost::numeric::odeint;
using namespace Constants;

CC_DDP_optimizer_new::CC_DDP_optimizer_new() {}

CC_DDP_optimizer_new::CC_DDP_optimizer_new(Cost_Function* c)
{
    Ru = c->get_control_cost_u();
    Rv = c->get_control_cost_v();
    Q_x = c->get_state_cost();
    Q_f = c->get_final_cost();
    x_target = c->get_target_state();

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

} // construct a CC_DDP_optimizer_new for the system passed as a parameter

CC_DDP_optimizer_new::~CC_DDP_optimizer_new() {}




/**
    This function quadratizes the cost along the given state and control trajectories: x_traj,u_traj,v_traj.
    It assigns the cost and its derivatives at time step i (t = dt * i) to the values passed in parameters 4-13
    in index i.

    @param x_traj - a vector of Eigen::VectorXd's corresponding to the state trajectory
    @param u_traj - a vector of Eigen::VectorXd's corresponding to the control_u trajectory
    @param v_traj - a vector of Eigen::VectorXd's corresponding to the control_v trajectory

*/
void CC_DDP_optimizer_new::quadratize_cost_mm(const vector<VectorXd>& x_traj, const vector<VectorXd>& u_traj, const vector<VectorXd>& v_traj)
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






/** This function takes in x_traj and Linearized dynamics A,B,C to back propagate the value ode "value_dynamics_mm" */
void CC_DDP_optimizer_new::backpropagate_mm_rk(const vector<VectorXd>& x_traj,
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
        integrate_adaptive(controlled_stepper, std::bind(&::CC_DDP_optimizer_new::value_dynamics_mm, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), V_std , t , t-dt , -dt );
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
void CC_DDP_optimizer_new::update_controls_mm(const vector<VectorXd>& dx_traj,
                                          vector<VectorXd>& u_traj, vector<VectorXd>& v_traj)
{
    // Clamping
    // if(!Constants::constraint.compare("clamp"))
    // {
    //     this->update_controls_clamp(dx_traj, u_traj, v_traj);
    // }
    // // Single constraint, only on u_traj
    // else if (!Constants::constraint.compare("single_box_qp"))
    // {
    //     this->update_controls_box_qp(dx_traj, u_traj, v_traj, true);
    // }
    // // Any custom constraint strategy
    // else if (!Constants::constraint.compare("custom"))
    // {
    //     // TODO: implement
    // }
    // // Box QP
    // else
    // {
        this->update_controls_box_qp(dx_traj, u_traj, v_traj);
    // }
}



void CC_DDP_optimizer_new::update_controls_box_qp(const vector<VectorXd>& dx_traj,
                                          vector<VectorXd>& u_traj, vector<VectorXd>& v_traj, bool single)
{
    // Vectors for delta u and delta v
    VectorXd du(num_controls_u);
    VectorXd dv(num_controls_v);

    // We have 4 controls for block matrix operations
    // TODO: make this dynamically chosen in configuration
    const int u_size = 4;

    // Quadratic program parameters
    Eigen::VectorXd cu, cv;                                     // c vectors
    Eigen::VectorXd b1(2*num_controls_u), b2(2*num_controls_u); // intermediate b vectors

    // Calculate quadratic matrix
    Eigen::MatrixXd D = MatrixXd::Zero(2*num_controls_u, 2*num_controls_u);
    D.block<u_size, u_size>(0,0) = Quu;
    // D.block<u_size,u_size>(0,u_size) = -Quv[0];
    // D.block<u_size,u_size>(u_size,0) = Quv[0].transpose();
    D.block<u_size,u_size>(u_size,u_size) = -Qvv;

    // Make D symmetric
    D = 0.5 * (D + D.transpose());

    cu = Qux_[0]*dx_traj[0] + Qu_[0];
    cv = -Qvx_[0]*dx_traj[0] - Qv_[0];
    Eigen::VectorXd c(cu.size() + cv.size());

    // Control Constraint DDP
    Program qp(CGAL::SMALLER);
    CGAL::Quadratic_program_options options;

    // QP options
    // options.set_verbosity(1);
    // options.set_auto_validation(true);
    qp.set_c0(0);

    // Set the quadratic term in the program
    for (int j = 0; j < Constants::num_controls_u*2; ++j)
    {
        for (int k = 0; k < Constants::num_controls_u*2; ++k)
        {
            qp.set_d(j, k, D(j, k));
        }
    }
    
    // Update the controls using a QP solver
    for (int i = 0; i < num_time_steps-1; i++) {
        /**
         * Double Control constraint DDP logic
         */

        // Calculate boundaries for du
        b1.segment(0, u_size) = Constants::u_lower - u_traj[i];
        b1.segment(u_size, u_size) = Constants::v_lower - v_traj[i];
        b2.segment(0, u_size) = Constants::u_upper - u_traj[i];
        b2.segment(u_size, u_size) = Constants::v_upper - v_traj[i];

        // Set control boundaries
        for(int j = 0; j < 2*num_controls_u; ++j)
        {
            qp.set_l(j, true, b1(j));
            qp.set_u(j, true, b2(j));
        }

        // Calculate linear objective function for du and dv
        cu = Qux_[i]*dx_traj[i] + Qu_[i];
        cv = -Qvx_[i]*dx_traj[i] - Qv_[i];

        // Concatenate these vectors into a matrix c
        c.segment(0, cu.size()) = cu;
        c.segment(cu.size(), cu.size()) = cv;

        // Add linear objective function to the program solver
        for (int j = 0; j < c.size(); ++j)
        {
            qp.set_c(j, c(j));
        }

        // solve quadratic program to find du
        Solution sol = CGAL::solve_quadratic_program(qp, ET());//, options);

        // Find du after performing the quadratic programming step if the solution is feasible
        if(sol.solves_quadratic_program(qp) && !sol.is_infeasible())
        {
            Solution::Variable_value_iterator it = sol.variable_values_begin();
            Solution::Variable_value_iterator end = sol.variable_values_end();

            for (int u = 0; it != end; ++it, ++u) {
                if (u < num_controls_u)
                {
                    du(u) = CGAL::to_double(*it);
                }
                else
                {
                    dv(u - num_controls_u) = CGAL::to_double(*it);
                }
            }
        }
        // QP failed, so use normal du and dv
        else
        {
            std::cout << "QP Solution not found!\n";
            dv = lv_[i] + Kv_[i] * dx_traj[i];
            du = lu_[i] + Ku_[i] * dx_traj[i];
        }

        // Update controls
        u_traj[i] = u_traj[i] + learning_rate * du;
        v_traj[i] = v_traj[i] + learning_rate * dv;
    }
}


void CC_DDP_optimizer_new::update_controls_clamp(const vector<VectorXd>& dx_traj,
                                          vector<VectorXd>& u_traj, vector<VectorXd>& v_traj)
{
    // Vectors for delta u and delta v
    VectorXd du(num_controls_u);
    VectorXd dv(num_controls_v);

    Eigen::VectorXd lower_u(num_controls_u), upper_u(num_controls_u); // upper and lower bounds for u
    Eigen::VectorXd lower_v(num_controls_v), upper_v(num_controls_v); // upper and lower bounds for v

    // Establish boundaries
    // du
    upper_u = Constants::u_upper;
    lower_u = Constants::u_lower;

    // dv
    upper_v = Constants::v_upper;
    lower_v = Constants::v_lower;


    // Update the controls using a clamping
    for (int i = 0; i < num_time_steps-1; i++) {
        // Update controls
        u_traj[i] = u_traj[i] + learning_rate * du;
        v_traj[i] = v_traj[i] + learning_rate * dv;

        // Clamping logic for u_traj
        for(int j = 0; j < Constants::num_controls_u; ++j)
        {
            u_traj[i](j) = this->clamp(u_traj[i](j), lower_u(j), upper_u(j));
        }
        // Clamping logic for v_traj
        for(int j = 0; j < Constants::num_controls_v; ++j)
        {
            v_traj[i](j) = this->clamp(v_traj[i](j), lower_v(j), upper_v(j));
        }
    }
}




double CC_DDP_optimizer_new::clamp(double val, double min, double max)
{
    if(val < min)
    {
        return min;
    }
    else if(val > max)
    {
        return max;
    }

    return val;
}
