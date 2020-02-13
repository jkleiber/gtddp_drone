#ifndef PURSUIT_COST_H
#define PURSUIT_COST_H

#include "../Cost_Function.h"

class PursuitCost : public Cost_Function{
    public:
        PursuitCost();
        PursuitCost(Eigen::VectorXd x_0);
        ~PursuitCost();

        void initialize_cost_matrix();
        double calculate_cost_mm(const std::vector<Eigen::VectorXd>&,
                                 const std::vector<Eigen::VectorXd>&,
                                 const std::vector<Eigen::VectorXd>&);
};

#endif
