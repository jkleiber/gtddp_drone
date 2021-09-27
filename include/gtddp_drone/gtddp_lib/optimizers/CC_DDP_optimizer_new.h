#ifndef CC_DDP_optimizer_new_H
#define CC_DDP_optimizer_new_H

#include "../DDP_Optimizer.h"

#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;

typedef CGAL::Quadratic_program<double> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;

class CC_DDP_optimizer_new : public DDP_Optimizer
{
    private:
        // du constraint
        std::vector<Eigen::MatrixXd> Qux_;
        std::vector<Eigen::MatrixXd> Quv_;
        std::vector<Eigen::MatrixXd> Qu_;

        // dv constraint
        std::vector<Eigen::MatrixXd> Qvx_;
        std::vector<Eigen::MatrixXd> Qv_;

        // Constrained control update functions
        void update_controls_box_qp(const std::vector<Eigen::VectorXd>&,std::vector<Eigen::VectorXd>&, std::vector<Eigen::VectorXd>&, bool single = false);
        void update_controls_clamp(const std::vector<Eigen::VectorXd>&,std::vector<Eigen::VectorXd>&, std::vector<Eigen::VectorXd>&);

        // Helper functions
        double clamp(double val, double min, double max);

    public:
        /**
            constructs a CC_DDP_optimizer_new for the given system
            @param c - the cost function this optimizer will use
        */
        CC_DDP_optimizer_new();
        CC_DDP_optimizer_new(Cost_Function* c);
        ~CC_DDP_optimizer_new();

        void backpropagate_mm_rk(const std::vector<Eigen::VectorXd>&,
                                 const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&);

        void update_controls_mm(const std::vector<Eigen::VectorXd>&,std::vector<Eigen::VectorXd>&, std::vector<Eigen::VectorXd>&);
        void quadratize_cost_mm(const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&, const std::vector<Eigen::VectorXd>&);
};

#endif // CC_DDP_optimizer_new_H
