#ifndef PURSUIT_OPTIMIZER_H
#define PURSUIT_OPTIMIZER_H

#include "../DDP_Optimizer.h"

#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;

typedef CGAL::Quadratic_program<double> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;

class Pursuit_optimizer : public DDP_Optimizer
{
    private:
        // du constraint
        std::vector<Eigen::MatrixXd> Qux_;
        std::vector<Eigen::MatrixXd> Quv_;
        std::vector<Eigen::MatrixXd> Qu_;

        // dv constraint
        std::vector<Eigen::MatrixXd> Qvx_;
        std::vector<Eigen::MatrixXd> Qv_;

    public:
        /**
            constructs a Pursuit_optimizer for the given system
            @param c - the cost function this optimizer will use
        */
        Pursuit_optimizer();
        Pursuit_optimizer(Cost_Function* c);
        ~Pursuit_optimizer();

        void backpropagate_mm_rk(const std::vector<Eigen::VectorXd>&,
                                 const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&);

        void update_controls_mm(const std::vector<Eigen::VectorXd>&,std::vector<Eigen::VectorXd>&, std::vector<Eigen::VectorXd>&);
        void quadratize_cost_mm(const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&);
};

#endif // PURSUIT_OPTIMIZER_H
