#ifndef COST_FUNCTION_H
#define COST_FUNCTION_H

#include <eigen3/Eigen/Dense>
#include <vector>

#include "Constants.h"

class Cost_Function
{
    public:
        virtual void initialize_cost_matrix() = 0;
        virtual double calculate_cost_mm(const std::vector<Eigen::VectorXd>&,
                                         const std::vector<Eigen::VectorXd>&,
                                         const std::vector<Eigen::VectorXd>&) = 0;

        void set_control_cost_u(Eigen::MatrixXd val) { Ru = val; }
		void set_control_cost_v(Eigen::MatrixXd val) { Rv = val; }
        void set_state_cost(Eigen::MatrixXd val) { Q_x = val; }
        void set_final_cost(Eigen::MatrixXd val) { Q_f = val; }
        void set_target_state(Eigen::VectorXd val) { x_target = val; }
        Eigen::MatrixXd get_control_cost_u() { return Ru; }
		Eigen::MatrixXd get_control_cost_v() { return Rv; }
        Eigen::MatrixXd get_state_cost() { return Q_x; }
        Eigen::MatrixXd get_final_cost() { return Q_f; }
        Eigen::VectorXd get_target_state() { return x_target; }


    protected:
        Eigen::MatrixXd Ru;
		Eigen::MatrixXd Rv;
        Eigen::MatrixXd Q_x;
        Eigen::MatrixXd Q_f;
        Eigen::VectorXd x_target;
};

#endif // COST_FUNCTION_H
