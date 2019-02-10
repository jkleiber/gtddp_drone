#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>
#include <fstream>
#include <cstdio>
#include <ctime>

#include "gtddp_lib/DDP_main_mm.h"

using namespace std;
using namespace Constants;
using namespace Eigen;
using namespace boost::numeric::odeint;

void print_trajectory(vector<VectorXd> traj)
{
    std::cout<< "trajectory " <<endl;
    for(int i=0; i<num_time_steps-1 ;i=i+5) {
        std::cout << traj[i].transpose() <<endl;
    }
    
} //print_traj


/**
    This function will run the Differential Dynamic Programming trajectory
    optimization algorithm
*/
int main()
{
    int start_s=clock();
    /*  BEGIN INITIAL AND TARGET STATE INITIALIZATION
        x_0 is num_states x 1 vector of initial states
        x_target is num_states x 1 vector of target states
    */
    VectorXd x_0(num_states);       // do not edit
    x_0 << 0,0,0,0,0,0,0,0,0,0,0,0;                 // initial state using comma initialization
//    x_0 << 0,0,pi,0;
    VectorXd x_target(num_states);  // do not edit
    x_target << 0,0,2,0,0,0,0,0,0,0,0,0;           // target state using comma initialization

    DDP_main_mm ddpmain(x_0,x_target);
    
    ddpmain.ddp_loop();
    int stop_s=clock();
    printf("Entire Execution Time: %i\n",stop_s-start_s );

    return 0;
}
