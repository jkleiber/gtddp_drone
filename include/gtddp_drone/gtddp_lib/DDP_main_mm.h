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

#include "Constants.h"
#include "GT_DDP_optimizer.h"
#include "Cost_Function.h"
#include "Quadrotor.h"      // include the system's header file (Quadrotor.h, Pendulum.h, etc.)

class DDP_main_mm
{
public:
    DDP_main_mm ();
    DDP_main_mm (Eigen::VectorXd x, Eigen::VectorXd x_t);
    ~DDP_main_mm();
    
    void ddp_loop();
    
    std::vector<Eigen::VectorXd> get_x_traj();
    std::vector<Eigen::VectorXd> get_u_traj();
    std::vector<Eigen::VectorXd> get_lu();
    std::vector<Eigen::MatrixXd> get_Ku();
    
private:

    Quadrotor quad;
    Cost_Function cost;
    GT_DDP_optimizer ddp;
    
    Eigen::VectorXd x_target;
    
    std::vector<Eigen::VectorXd> x_traj, u_traj, v_traj, dx_traj;
    std::vector<Eigen::MatrixXd> A, B, C;
    std::vector<double> trajectory_cost_mm;
    
};




#endif /* DDP_MAIN_MM_H */
