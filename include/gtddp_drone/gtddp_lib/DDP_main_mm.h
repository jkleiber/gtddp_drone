//
//  DDP_main_mm.hpp
//  GTDDP
//
//  Created by 임재인 on 2018. 2. 14..
//  Copyright © 2018년 jaein. All rights reserved.
//

#ifndef DDP_MAIN_MM_H
#define DDP_MAIN_MM_H

#include <stdio.h>
#include <eigen3/Eigen/Dense>
#include <vector>

#include "Constants.h"

// Cost Functions
#include "Cost_Function.h"
#include "cost_functions/SingleQuadrotorCost.h"
#include "cost_functions/PursuitCost.h"

// Optimizers
#include "DDP_Optimizer.h"
#include "optimizers/CC_DDP_optimizer.h"
#include "optimizers/GT_DDP_optimizer.h"
#include "optimizers/Pursuit_optimizer.h"

// Systems
#include "Drone.h"
#include "systems/Quadrotor.h"
#include "systems/PursuitDrones.h"
#include "systems/CartPole.h"

class DDP_main_mm
{
public:
    DDP_main_mm ();
    DDP_main_mm (Eigen::VectorXd x, Eigen::VectorXd x_t);
    ~DDP_main_mm();

    void update(Eigen::VectorXd x, Eigen::VectorXd x_t);
    void ddp_loop();

    std::vector<Eigen::VectorXd> get_x_traj();
    std::vector<Eigen::VectorXd> get_u_traj();
    std::vector<Eigen::VectorXd> get_v_traj();
    std::vector<Eigen::VectorXd> get_lu();
    std::vector<Eigen::MatrixXd> get_Ku();
    std::vector<Eigen::MatrixXd> get_Kv();

    void print_trajectory(std::vector<Eigen::VectorXd> traj);

private:

    Drone *drone;
    Cost_Function *cost;
    DDP_Optimizer *ddp;

    Eigen::VectorXd x_target;

    std::vector<Eigen::VectorXd> x_traj, u_traj, v_traj, dx_traj;
    std::vector<Eigen::MatrixXd> A, B, C;
    std::vector<double> trajectory_cost_mm;

    int num_legs;
    int max_iterations;

};




#endif /* DDP_MAIN_MM_H */
