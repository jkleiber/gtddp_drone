//
//  DDP_main_mm.cpp
//  GTDDP
//
//  Created by 임재인 on 2018. 2. 14..
//  Copyright © 2018년 jaein. All rights reserved.
//

//#include "Constants.h"
#include <ctime>

#include "gtddp_drone/gtddp_lib/DDP_main_mm.h"

using namespace Constants;
DDP_main_mm::DDP_main_mm()
{
    // Choose the system based on the program selector
    if(!Constants::ddp_selector.compare("pursuit"))
    {
        drone = new PursuitDrones();
    }
    else
    {
        drone = new Quadrotor();
    }

    //  ***** DO NOT EDIT *****
//  Initialize state and control trajectories, dF_dx, dF_du, dF_dv
    x_traj.resize(num_time_steps);
    u_traj.resize(num_time_steps - 1);
    v_traj.resize(num_time_steps - 1);

    A.resize(num_time_steps - 1);
    B.resize(num_time_steps - 1);
    C.resize(num_time_steps - 1);
    dx_traj.resize(num_time_steps);

    trajectory_cost_mm.resize(num_iterations);

    ddp->initialize_trajectories(x_traj, u_traj, v_traj, dx_traj);

    //Set the initial max iterations to the high value and initialize the leg counter
    this->max_iterations = Constants::num_iterations;
    this->num_legs = 0;
}

DDP_main_mm::DDP_main_mm(Eigen::VectorXd x, Eigen::VectorXd x_t)
{
    // Choose the system based on the program selector
    if(!Constants::ddp_selector.compare("pursuit"))
    {
        drone = new PursuitDrones();
    }
    else
    {
        drone = new Quadrotor();
    }

    if(!Constants::ddp_selector.compare("pursuit"))
    {
        cost = new PursuitCost();
    }
    else
    {
        cost = new SingleQuadrotorCost(x_t);
    }

    // If the DDP is constrained, use the CC_DDP_optimizer
    if(!Constants::ddp_selector.compare("ccddp"))
    {
        ddp = new CC_DDP_optimizer(cost);
    }
    // If this is a pursuit scenario, use the pursuit optimizer
    else if(!Constants::ddp_selector.compare("pursuit"))
    {
        ddp = new Pursuit_optimizer(cost);
    }
    // Otherwise use the default GT_DDP_optimizer
    else
    {
        ddp = new GT_DDP_optimizer(cost);
    }



//  ***** DO NOT EDIT *****
//  Initialize state and control trajectories, dF_dx, dF_du, dF_dv
    x_traj.resize(num_time_steps);
    u_traj.resize(num_time_steps - 1);
    v_traj.resize(num_time_steps - 1);

    A.resize(num_time_steps - 1);
    B.resize(num_time_steps - 1);
    C.resize(num_time_steps - 1);
    dx_traj.resize(num_time_steps);

    trajectory_cost_mm.resize(num_iterations);

    ddp->initialize_trajectories(x_traj, u_traj, v_traj, dx_traj);
    x_traj[0]=x;

    //Set the initial max iterations to the high value and initialize the leg counter
    this->max_iterations = Constants::num_iterations;
    this->num_legs = 0;

}

DDP_main_mm::~DDP_main_mm() {}


void DDP_main_mm::update(Eigen::VectorXd x, Eigen::VectorXd x_t)
{
    if(!Constants::ddp_selector.compare("pursuit"))
    {
        cost = new PursuitCost();
    }
    else
    {
        cost = new SingleQuadrotorCost(x_t);
    }

    // If the DDP is constrained, use the CC_DDP_optimizer
    if(!Constants::ddp_selector.compare("ccddp"))
    {
        ddp = new CC_DDP_optimizer(cost);
    }
    else if(!Constants::ddp_selector.compare("pursuit"))
    {
        ddp = new Pursuit_optimizer(cost);
    }
    // Otherwise use the default GT_DDP_optimizer
    else
    {
        ddp = new GT_DDP_optimizer(cost);
    }

    //Reset x_traj to 0
    for (int i = 0; i < num_time_steps; i++) {
        x_traj[i] = Eigen::VectorXd::Zero(num_states);
    }

    //Update current trajectory
    x_traj[0]=x;
}

void DDP_main_mm::ddp_loop()
{
    //Increment the leg counter
    ++num_legs;

    //If the leg counter has exceeded the number of long legs required before short iterations,
    //then switch to short iterations
    if(num_legs > Constants::num_long_legs)
    {
        this->max_iterations = Constants::short_iterations;
    }

    //int start_s=clock();
    for (int i = 0; i < max_iterations; i++)
    {
        // Play the controls on the real dynamics to generate new x_traj, u_traj, v_traj //
        drone->forward_propagate_mm(x_traj, u_traj, v_traj);
        //this->print_trajectory(this->get_x_traj());

        std::cout << "1: " << x_traj[0].transpose() << std::endl;

        // Output the cost
        trajectory_cost_mm[i] = cost->calculate_cost_mm(x_traj, u_traj, v_traj);
        printf("Quadrotor DDP 0\tIteration %i\tCost = %10.4f\n", i + 1, trajectory_cost_mm[i]);

        std::cout << "2: " << x_traj[0].transpose()  << std::endl;

        // Linearize dynamics along x_traj, u_traj, v_traj
        drone->linearize_dynamics_mm(x_traj, u_traj, v_traj, A, B, C);

        std::cout << "3: " << x_traj[0].transpose()  << std::endl;

        // Linearize cost along x_traj, u_traj, v_traj
        ddp->quadratize_cost_mm(x_traj, u_traj, v_traj);

        std::cout << "4: " << x_traj[0].transpose()  << std::endl;

        // Backpropagate the value function and its derivatives along x_traj, u_traj
        ddp->backpropagate_mm_rk(x_traj, A, B, C);

        std::cout << "5: " << x_traj[0].transpose()  << std::endl;

        // forward propagate the dynamics along x_traj, u_traj, v_traj to get dx_traj
        ddp->forward_propagate_mm_rk(dx_traj, A, B, C);

        std::cout << "6: " << x_traj[0].transpose()   << std::endl;

        // Update the controls
        ddp->update_controls_mm(dx_traj, u_traj, v_traj);

        std::cout << "7: " << x_traj[0].transpose()   << std::endl;

        ////
    } // END MAIN DDP LOOP
    //int stop_s=clock();
    //printf("DDP Loop Execution Time: %i\n",stop_s-start_s );
    //printf("b\n");
} //ddp_loop()


std::vector<Eigen::VectorXd> DDP_main_mm::get_x_traj(){
    return x_traj;
}
std::vector<Eigen::VectorXd> DDP_main_mm::get_u_traj(){

    return u_traj;
}
std::vector<Eigen::VectorXd> DDP_main_mm::get_v_traj(){

    return v_traj;
}
std::vector<Eigen::VectorXd> DDP_main_mm::get_lu(){

    return ddp->lu_;
}
std::vector<Eigen::MatrixXd> DDP_main_mm::get_Ku(){

    return ddp->Ku_;
}
std::vector<Eigen::MatrixXd> DDP_main_mm::get_Kv(){
    return ddp->Kv_;
}

void DDP_main_mm::print_trajectory(std::vector<Eigen::VectorXd> traj)
{
    std::cout<< "trajectory \n";
    for(int i=0; i<traj.size() ;i=i+5) {
        std::cout << traj[i].transpose() << std::endl;
    }

}
