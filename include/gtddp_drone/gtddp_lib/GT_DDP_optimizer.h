#ifndef GT_DDP_OPTIMIZER_H
#define GT_DDP_OPTIMIZER_H

#include "DDP_Optimizer.h"

class GT_DDP_optimizer : public DDP_Optimizer
{
    public:
        /**
            constructs a GT_DDP_optimizer for the given system
            @param c - the cost function this optimizer will use
        */
        GT_DDP_optimizer();
        GT_DDP_optimizer(Cost_Function c);
        ~GT_DDP_optimizer();

        void backpropagate_mm_rk(const std::vector<Eigen::VectorXd>&,
                                 const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&);

        void update_controls_mm(const std::vector<Eigen::VectorXd>&,std::vector<Eigen::VectorXd>&, std::vector<Eigen::VectorXd>&);

};

#endif // DDP_OPTIMIZER_H
