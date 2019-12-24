#ifndef CC_DDP_OPTIMIZER_H
#define CC_DDP_OPTIMIZER_H

#include "DDP_Optimizer.h"

#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;

typedef CGAL::Quadratic_program<double> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;

class CC_DDP_optimizer : public DDP_Optimizer
{
    private:
        std::vector<Eigen::MatrixXd> Qux_;
        std::vector<Eigen::MatrixXd> Quv_;
        std::vector<Eigen::MatrixXd> Qu_;


    public:
        /**
            constructs a CC_DDP_optimizer for the given system
            @param c - the cost function this optimizer will use
        */
        CC_DDP_optimizer();
        CC_DDP_optimizer(Cost_Function c);
        ~CC_DDP_optimizer();

        void backpropagate_mm_rk(const std::vector<Eigen::VectorXd>&,
                                 const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&);

        void update_controls_mm(const std::vector<Eigen::VectorXd>&,std::vector<Eigen::VectorXd>&, std::vector<Eigen::VectorXd>&);
};

#endif // CC_DDP_OPTIMIZER_H
