#ifndef SINGLE_QUADROTOR_COST_H
#define SINGLE_QUADROTOR_COST_H

#include "../Cost_Function.h"

class SingleQuadrotorCost : public Cost_Function{
    public:
        SingleQuadrotorCost();
        SingleQuadrotorCost(Eigen::VectorXd x_t);
        ~SingleQuadrotorCost();

        void initialize_cost_matrix();
        double calculate_cost_mm(const std::vector<Eigen::VectorXd>&,
                                 const std::vector<Eigen::VectorXd>&,
                                 const std::vector<Eigen::VectorXd>&);
};

#endif
