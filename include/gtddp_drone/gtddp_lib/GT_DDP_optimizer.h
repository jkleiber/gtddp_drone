#ifndef GT_DDP_OPTIMIZER_H
#define GT_DDP_OPTIMIZER_H

#include <eigen3/Eigen/Dense>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include "Constants.h"
#include "Cost_Function.h"

#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;

typedef CGAL::Quadratic_program<double> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;

class GT_DDP_optimizer
{

private:
    Cost_Function cost;
    Eigen::MatrixXd Ru, Rv, Q_f, Q_x;
    Eigen::VectorXd x_target;

    // stepper for forward propagating & contolled_stepper for backward propagating
    boost::numeric::odeint::runge_kutta_dopri5<Eigen::VectorXd, double, Eigen::VectorXd, double, boost::numeric::odeint::vector_space_algebra> stepper;
    typedef boost::numeric::odeint::runge_kutta_cash_karp54<std::vector<double> > error_stepper_type;
//    typedef boost::numeric::odeint::runge_kutta_dopri5<std::vector<double>> error_stepper_type;
    typedef boost::numeric::odeint::controlled_runge_kutta< error_stepper_type > controlled_stepper_type;
    controlled_stepper_type controlled_stepper;


    //temporary data members for Value backpropagating: They will be saved in the public std::vector.
    double L_0i_;
    Eigen::VectorXd L_xi_;
    Eigen::VectorXd L_ui_;
    Eigen::VectorXd L_vi_;
    Eigen::MatrixXd L_xxi_;
    Eigen::MatrixXd L_uui_;
    Eigen::MatrixXd L_vvi_;
    Eigen::MatrixXd L_uxi_;
    Eigen::MatrixXd L_vxi_;
    Eigen::MatrixXd L_uvi_;

    Eigen::MatrixXd Ai_;
    Eigen::MatrixXd Bi_;
    Eigen::MatrixXd Ci_;

    //    ****** Feedback and Feedforward gains  ******
    //    lu(num_time_steps-1), lv(num_time_steps - 1);
    //    Ku(num_time_steps-1), Kv(num_time_steps - 1);

    Eigen::VectorXd lui_;
    Eigen::VectorXd lvi_;
    Eigen::MatrixXd Kui_;
    Eigen::MatrixXd Kvi_;

    //ode solver for value back propagation
    void value_dynamics_mm(const std::vector<double>& V_pkg , std::vector<double>& dV_pkg,  double );


    //vectors of num_time_steps to keep quadratized costs over horizon
    std::vector<double> L_0_;
    std::vector<Eigen::VectorXd> L_x_;
    std::vector<Eigen::VectorXd> L_u_;
    std::vector<Eigen::VectorXd> L_v_;
    std::vector<Eigen::MatrixXd> L_xx_;
    std::vector<Eigen::MatrixXd> L_uu_;
    std::vector<Eigen::MatrixXd> L_vv_;
    std::vector<Eigen::MatrixXd> L_ux_;
    std::vector<Eigen::MatrixXd> L_vx_;
    std::vector<Eigen::MatrixXd> L_uv_;


    Eigen::VectorXd Qx;
    Eigen::VectorXd Qu;
    Eigen::VectorXd Qv;
    Eigen::MatrixXd Qxx;
    Eigen::MatrixXd Quu;
    Eigen::MatrixXd Qvv;
    Eigen::MatrixXd Qux;
    Eigen::MatrixXd Qvx;
    Eigen::MatrixXd Quv;
    Eigen::MatrixXd Qvu;
    Eigen::MatrixXd Quu_inv;
    Eigen::MatrixXd Qvv_inv;
    Eigen::MatrixXd G;
    Eigen::MatrixXd H;
    Eigen::MatrixXd G_inv;
    Eigen::MatrixXd H_inv;

    // du
    std::vector<Eigen::MatrixXd> Qux_;
    std::vector<Eigen::MatrixXd> Quv_;
    std::vector<Eigen::MatrixXd> Qu_;

    // dv
    std::vector<Eigen::MatrixXd> Qvx_;
    std::vector<Eigen::MatrixXd> Qv_;

public:
    /**
        constructs a GT_DDP_optimizer for the given system
        @param c - the cost function this optimizer will use
    */
    GT_DDP_optimizer();
    GT_DDP_optimizer(Cost_Function c);
    ~GT_DDP_optimizer();

    //vectors to keep DDP gains over horizon
    std::vector<Eigen::VectorXd> lu_;
    std::vector<Eigen::VectorXd> lv_;
    std::vector<Eigen::MatrixXd> Ku_;
    std::vector<Eigen::MatrixXd> Kv_;

    void quadratize_cost_mm(const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&);
    void backpropagate_mm_rk(const std::vector<Eigen::VectorXd>&,
                             const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&);

    void forward_propagate_mm_rk(std::vector<Eigen::VectorXd>&,
                             const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&);
    void update_controls_mm(const std::vector<Eigen::VectorXd>&,std::vector<Eigen::VectorXd>&, std::vector<Eigen::VectorXd>&);

    void initialize_trajectories_to_zero_mm(std::vector<Eigen::VectorXd>&, std::vector<Eigen::VectorXd>&, std::vector<Eigen::VectorXd>&,std::vector<Eigen::VectorXd>&);

};

#endif // DDP_OPTIMIZER_H
