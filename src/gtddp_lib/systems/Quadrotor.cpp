
#include <iostream>
#include <cmath>

#include "gtddp_drone/gtddp_lib/systems/Quadrotor.h"

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
void Quadrotor::dynamics_mm(const Eigen::VectorXd& x , Eigen::VectorXd& dxdt,  double t)
{

    double x6=x(6);
    double x7=x(7);
    double x8=x(8);
    double x9=x(9);
    double x10=x(10);
    double x11=x(11);

    double sin_phi = sin(x6);
    double cos_phi = cos(x6);

    double sin_theta = sin(x7);
    double cos_theta = cos(x7);
    double tan_theta = tan(x7);
    double sec_theta = 1/cos_theta;

    double sin_psi = sin(x8);
    double cos_psi = cos(x8);

    double u0 = ui_(0);
    double u1 = ui_(1);
    double u2 = ui_(2);
    double u3 = ui_(3);

    double v0 = vi_(0);
    double v1 = vi_(1);
    double v2 = vi_(2);
    double v3 = vi_(3);

    //shallow copy
    VectorXd dx(num_states);
    dx.setZero();

    dx(0)=x(3);
    dx(1)=x(4);
    dx(2)=x(5);
    dx(3)=((u0 + v0)*(cos_phi*cos_psi*sin_theta + sin_phi*sin_psi))/m;
    dx(4)=((u0 + v0)*(-(cos_psi*sin_phi) + cos_phi*sin_theta*sin_psi))/m;
    dx(5)= -grav + ((u0 + v0)*cos_phi*cos_theta)/m;
    dx(6)=x9 + x11*cos_phi*tan_theta + x10*sin_phi*tan_theta;
    dx(7)=x10*cos_phi - x11*sin_phi;
    dx(8)=x11*cos_phi*sec_theta + x10*sec_theta*sin_phi;
    dx(9)=(u1 + v1 + Iyy*x10*x11 - Izz*x10*x11)/Ixx;
    dx(10)=(u2 + v2 - Ixx*x11*x9 + Izz*x11*x9)/Iyy;
    dx(11)=(u3 + v3 + Ixx*x10*x9 - Iyy*x10*x9)/Izz;

    dxdt=dx;

}

/**
 forward_propagate_mm calls ode object dynamics_mm at each iteration to propagate
 dynamics given u_traj, v_traj
 @param u_traj_ - a num_controls x 1 VectorXd of stabilizing controls
 @param v_traj - a num_controls x 1 VectorXd of destabilizing controls
 @update x_traj - update x_traj
 */
void Quadrotor::forward_propagate_mm(vector<VectorXd>& x_traj, const vector<VectorXd>& u_traj, const vector<VectorXd>& v_traj)
{
    // initial condition
    VectorXd x= x_traj[0];
    double t=0;
    // main integration loop : propagation over time step
    for (int i = 0; i < num_time_steps-1; i++) {
       //update current inputs
        ui_=u_traj[i];
        vi_=v_traj[i];

        using namespace std::placeholders;
        stepper.do_step(std::bind(&::Quadrotor::dynamics_mm, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), x, t, dt);
        t += dt;
        x_traj[i+1] = x; //save x trajectory
        t += dt;
    } //for num_time_steps-1

} //forward_propagate_mm


void Quadrotor::feedforward_controls(VectorXd current_state, DroneTrajectory& drone_traj)
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
        ui_ = drone_traj.u_traj[i] + drone_traj.Ku_traj[i] * (x - drone_traj.x_traj[i]);
        vi_ = v_zero;

        stepper.do_step(std::bind(&::Quadrotor::dynamics_mm, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), x, t, dt);
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
void Quadrotor::linearize_dynamics_mm(const vector<VectorXd>& x_traj, const vector<VectorXd>& u_traj, const vector<VectorXd>& v_traj,
                                      vector<MatrixXd>& A,vector<MatrixXd>& B,vector<MatrixXd>& C){

    for (int i=0;i<num_time_steps-1;i++){
        double x6=x_traj[i](6);
        double x7=x_traj[i](7);
        double x8=x_traj[i](8);
        double x9=x_traj[i](9);
        double x10=x_traj[i](10);
        double x11=x_traj[i](11);

        double sin_phi = sin(x6);
        double cos_phi = cos(x6);

        double sin_theta = sin(x7);
        double cos_theta = cos(x7);
        double tan_theta = tan(x7);
        double sec_theta = 1/cos_theta;

        double sin_psi = sin(x8);
        double cos_psi = cos(x8);

        double u0 = u_traj[i](0);
        double v0 = v_traj[i](0);

        MatrixXd A_sec;
        MatrixXd B_sec;
        MatrixXd C_sec;
        A_sec.setZero(num_states,num_states);
        B_sec.setZero(num_states,num_controls_u);
        C_sec.setZero(num_states,num_controls_v);

        A_sec(0,3)=1;
        A_sec(1,4)=1;
        A_sec(2,5)=1;
        A_sec(3,6)=((u0 + v0)*(-(cos_psi*sin_phi*sin_theta) + cos_phi*sin_psi))/m;
        A_sec(3,7)=((u0 + v0)*cos_phi*cos_theta*cos_psi)/m;
        A_sec(3,8)=((u0 + v0)*(cos_psi*sin_phi - cos_phi*sin_theta*sin_psi))/m;
        A_sec(4,6)=-(((u0 + v0)*(cos_phi*cos_psi + sin_phi*sin_theta*sin_psi))/m);
        A_sec(4,7)=((u0 + v0)*cos_phi*cos_theta*sin_psi)/m;
        A_sec(4,8)=((u0 + v0)*(cos_phi*cos_psi*sin_theta + sin_phi*sin_psi))/m;
        A_sec(5,6)=-(((u0 + v0)*cos_theta*sin_phi)/m);
        A_sec(5,7)=-(((u0 + v0)*cos_phi*sin_theta)/m);
        A_sec(6,6)=(x10*cos_phi - x11*sin_phi)*tan_theta;
        A_sec(6,7)=pow(sec_theta,2)*(x11*cos_phi + x10*sin_phi);
        A_sec(6,9)=1;
        A_sec(6,10)=sin_phi*tan_theta;
        A_sec(6,11)=cos_phi*tan_theta;
        A_sec(7,6)=-(x11*cos_phi) - x10*sin_phi;
        A_sec(7,10)=cos_phi;
        A_sec(7,11)=-sin_phi;
        A_sec(8,6)=sec_theta*(x10*cos_phi - x11*sin_phi);
        A_sec(8,7)=sec_theta*(x11*cos_phi + x10*sin_phi)*tan_theta;
        A_sec(8,10)=sec_theta*sin_phi;
        A_sec(8,11)=cos_phi*sec_theta;
        A_sec(9,10)=((Iyy - Izz)*x11)/Ixx;
        A_sec(9,11)=((Iyy - Izz)*x10)/Ixx;
        A_sec(10,9)=((-Ixx + Izz)*x11)/Iyy;
        A_sec(10,11)=((-Ixx + Izz)*x9)/Iyy;
        A_sec(11,9)=((Ixx - Iyy)*x10)/Izz;
        A_sec(11,10)=((Ixx - Iyy)*x9)/Izz;

        B_sec(3,0)=(cos_phi*cos_psi*sin_theta + sin_phi*sin_psi)/m;
        B_sec(4,0)=(-(cos_psi*sin_phi) + cos_phi*sin_theta*sin_psi)/m;
        B_sec(5,0)=(cos_phi*cos_theta)/m;
        B_sec(9,1)=1/Ixx;
        B_sec(10,2)=1/Iyy;
        B_sec(11,3)=1/Izz;

        C_sec(3,0)=(cos_phi*cos_psi*sin_theta + sin_phi*sin_psi)/m;
        C_sec(4,0)=(-(cos_psi*sin_phi) + cos_phi*sin_theta*sin_psi)/m;
        C_sec(5,0)=(cos_phi*cos_theta)/m;
        C_sec(9,1)=1/Ixx;
        C_sec(10,2)=1/Iyy;
        C_sec(11,3)=1/Izz;

        A[i]=A_sec;
        B[i]=B_sec;
        C[i]=C_sec;

        //B_sec(3, 0) = (cos_psi * sin_theta + cos_theta * sin_phi * sin_psi) / m;
        //B_sec(4, 0) = (sin_psi * sin_theta - cos_theta * sin_phi * cos_psi) / m;
        //B_sec(5, 0) = (cos_phi * cos_theta) / m;
        //B_sec(9, 1)  = 1 / Ixx;
        //B_sec(10, 2) = 1 / Iyy;
        //B_sec(11, 3) = 1 / Izz;
        //
        //A[i]=A_sec;
        //B[i]=B_sec;
        //C[i]=B_sec;

    } //for (int i)
}// linearize_dynamics
