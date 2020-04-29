#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>

#include "gtddp_drone/gtddp_lib/systems/CartPole.h"

using namespace Eigen;
using namespace Constants;
using namespace std;
using namespace boost::numeric::odeint;


/**
Dynamics_mm is the main ode function to solve dx/dt = f(x,u,v).
@param x -  a num_states x 1 VectorXd of state values
@param u_ - a num_controls x 1 VectorXd of stabilizing controls
@param v_ - a num_controls x 1 VectorXd of destabilizing controls
@return updated x - a num_states x 1 VectorXd of updated state
*/
void CartPole::dynamics_mm(const Eigen::VectorXd& x , Eigen::VectorXd& dxdt,  double t)
{
    double x_dot = x(1);
    double theta = x(2);
    double theta_dot = x(3);
    double sin_theta=sin(theta);
    double cos_theta=cos(theta);
    double f = u_(0);
    double g = v_(0); //v(0)

    //shallow copy
    VectorXd dx(num_states);
    dx.setZero();
    dx(0) =  x_dot;
    dx(1) =  (-m*length*pow(theta_dot,2)*sin_theta + m*grav*sin_theta*cos_theta + (f+g))/(M+m*pow(sin_theta,2));
    dx(2) =  theta_dot;
    dx(3) =  (-m*length*pow(theta_dot,2)*sin_theta*cos_theta+ (M+m)*grav*sin_theta + (f+g)*cos_theta)/(M*length+m*length*pow(sin_theta,2));
    dxdt=dx;
}



/**
 forward_propagate_mm calls ode object dynamics_mm at each iteration to propagate
 dynamics given u_traj, v_traj
 @param u_traj_ - a num_controls x 1 VectorXd of stabilizing controls
 @param v_traj - a num_controls x 1 VectorXd of destabilizing controls
 @update x_traj - update x_traj
 */
void CartPole::forward_propagate_mm(vector<VectorXd>& x_traj, const vector<VectorXd>& u_traj, const vector<VectorXd>& v_traj)
{
    // initial condition
    VectorXd x = x_traj[0];
    double t = 0;

    // propagation over time step
    for (int i = 0; i < num_time_steps-1; i++) {
        u_ = u_traj[i];
        v_ = v_traj[i];

        // Propagate dynamics over num_time_steps
        stepper.do_step(std::bind(&::CartPole::dynamics_mm, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), x, t, dt);

        // save the resulting x trajectory and increment the time
        t += dt;
        x_traj[i+1] = x;
        t += dt;
    }

} //forward_propagate_mm



void CartPole::feedforward_controls(VectorXd current_state, DroneTrajectory& drone_traj)
{
    // initial condition
    VectorXd x = current_state;
    VectorXd v_zero(4);
    v_zero << 0, 0, 0, 0;

    // Result
    deque<VectorXd> result = drone_traj.x_traj;

    double t=0;
    // main integration loop : propagation over time step
    for (int i = 0; i < Constants::num_time_steps - 1; i++) {
        // update current inputs
        u_ = drone_traj.u_traj[i] + drone_traj.Ku_traj[i] * (x - drone_traj.x_traj[i]);
        v_ = v_zero;

        stepper.do_step(std::bind(&::CartPole::dynamics_mm, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), x, t, dt);
        t += dt;
        result[i] = x; //save x trajectory
        t += dt;
    } //for num_time_steps-1

    // Set x_traj to the new state
    drone_traj.x_traj = result;

} //forward_propagate_mm


/**
 linearize_dynamics_mm obtains the gradient of f (A,B,C) at the given trajectory
 @param x_traj -  a num_states x 1 VectorXd of state values
 @param u_traj_ - a num_controls x 1 VectorXd of stabilizing controls
 @param v_traj - a num_controls x 1 VectorXd of destabilizing controls
 @return A,B,C - gradient Matrix Fx Fu Fv
 */
void CartPole::linearize_dynamics_mm(const vector<VectorXd>& x_traj, const vector<VectorXd>& u_traj, const vector<VectorXd>& v_traj,
                                      vector<MatrixXd>& A,vector<MatrixXd>& B,vector<MatrixXd>& C){

    for (int i=0;i<num_time_steps-1;i++){
        double theta=x_traj[i](2);
        double theta_dot=x_traj[i](3);
        double f=u_traj[i](0);
        double g=v_traj[i](0);

        double sin_theta=sin(theta);
        double cos_theta=cos(theta);

        MatrixXd A_sec;
        MatrixXd B_sec;
        MatrixXd C_sec;
        A_sec.setZero(num_states,num_states);
        B_sec.setZero(num_states,num_controls_u);
        C_sec.setZero(num_states,num_controls_v);

        A_sec(0,1)=1;
        A_sec(1,2)=((-m*length*pow(theta_dot,2)*cos_theta + m*grav*(pow(cos_theta,2) - pow(sin_theta,2)))*(M+m*pow(sin_theta,2))- (-m*length*pow(theta_dot,2)*sin_theta + m*grav*sin_theta*cos_theta + (f+g))*2*m*sin_theta*cos_theta)/pow(M + m*pow(sin_theta,2),2);
        A_sec(1,3)=-2*m*length*theta_dot*sin_theta/(M + m*pow(sin_theta,2));
        A_sec(2,3)=1;
        A_sec(3,2)=((-m*length*pow(theta_dot,2)*(pow(cos_theta,2) - pow(sin_theta,2)) + (M+m)*grav*cos_theta - (f+g)*sin_theta)*length*(M + m*pow(sin_theta,2))- (-m*length*pow(theta_dot,2)*sin_theta*cos_theta + (M+m)*grav*sin_theta + (f+g)*cos_theta)*length*2*m*sin_theta*cos_theta)/pow(length*(M + m*pow(sin_theta,2)),2);
        A_sec(3,3)=-2*m*theta_dot*sin_theta*cos_theta/(M + m*pow(sin_theta,2));


        B_sec(1,0)=1/(M+m*pow(sin_theta,2));
        B_sec(3,0)=cos_theta/(length*(M+m*pow(sin_theta,2)));

        C_sec=B_sec;

        A[i]=A_sec;
        B[i]=B_sec;
        C[i]=C_sec;

    } //for (int i)
}// linearize_dynamics
