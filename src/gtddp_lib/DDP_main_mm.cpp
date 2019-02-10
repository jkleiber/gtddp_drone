//
//  DDP_main_mm.cpp
//  GTDDP
//
//  Created by 임재인 on 2018. 2. 14..
//  Copyright © 2018년 jaein. All rights reserved.
//

//#include "Constants.h"
#include <ctime>

#include "DDP_main_mm.h"

using namespace Constants;
DDP_main_mm::DDP_main_mm(Eigen::VectorXd x, Eigen::VectorXd x_t)
{

    quad = Quadrotor();
    cost = Cost_Function(x_t);
    ddp = GT_DDP_optimizer(cost);
    
//  ***** DO NOT EDIT *****
//  Initialize state and control trajectories, dF_dx, dF_du, dF_dv
    x_traj.resize(num_time_steps);
    u_traj.resize(num_time_steps-1);
    v_traj.resize(num_time_steps - 1);

    A.resize(num_time_steps-1);
    B.resize(num_time_steps-1);
    C.resize(num_time_steps - 1);
    dx_traj.resize(num_time_steps-1);
    
    trajectory_cost_mm.resize(num_iterations);
    
    ddp.initialize_trajectories_to_zero_mm(x_traj, u_traj, v_traj, dx_traj);
    x_traj[0]=x;
    
}

DDP_main_mm::~DDP_main_mm() {}
    
void DDP_main_mm::ddp_loop() 
{
    //int start_s=clock();
    for (int i = 0; i < num_iterations; i++) 
    {        
        // Play the controls on the real dynamics to generate new x_traj, u_traj, v_traj //
        quad.forward_propagate_mm(x_traj, u_traj, v_traj);
        
        // Output the cost
        trajectory_cost_mm[i] = cost.calculate_cost_mm(x_traj, u_traj, v_traj);
        printf("Cart Pole DDP\tIteration %i\tCost = %10.4f\n", i + 1, trajectory_cost_mm[i]);
        
        // Linearize dynamics along x_traj, u_traj, v_traj
        quad.linearize_dynamics_mm(x_traj, u_traj, v_traj, A, B, C);
        
        // Linearize cost along x_traj, u_traj, v_traj
        ddp.quadratize_cost_mm(x_traj, u_traj, v_traj);
        
        // Backpropagate the value function and its derivatives along x_traj, u_traj
        ddp.backpropagate_mm_rk(x_traj, A, B, C);
        
        // forward propagate the dynamics along x_traj, u_traj, v_traj to get dx_traj
        ddp.forward_propagate_mm_rk(dx_traj, A, B, C);
        
        // Update the controls
        ddp.update_controls_mm(dx_traj, u_traj, v_traj);
        //print_traj(ddp.get_lu());
        ////
    } // END MAIN DDP LOOP
    //int stop_s=clock();
    //printf("DDP Loop Execution Time: %i\n",stop_s-start_s );

} //ddp_loop()


std::vector<Eigen::VectorXd> DDP_main_mm::get_x_traj(){
    return x_traj;
}
std::vector<Eigen::VectorXd> DDP_main_mm::get_u_traj(){
    
    return u_traj;
}
std::vector<Eigen::VectorXd> DDP_main_mm::get_lu(){
    
    return ddp.lu_;
}
std::vector<Eigen::MatrixXd> DDP_main_mm::get_Ku(){
    
    return ddp.Ku_;
}
