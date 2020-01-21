
#include <iostream>
#include <cmath>

#include "gtddp_drone/gtddp_lib/systems/PursuitDrones.h"

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
void PursuitDrones::dynamics_mm(const Eigen::VectorXd& x, Eigen::VectorXd& dxdt, double t)
{
    // Drone 1 (u)
    double x6=x(6);
    double x7=x(7);
    double x8=x(8);
    double x9=x(9);
    double x10=x(10);
    double x11=x(11);

    double sin_phi_u = sin(x6);
    double cos_phi_u = cos(x6);

    double sin_theta_u = sin(x7);
    double cos_theta_u = cos(x7);
    double tan_theta_u = tan(x7);
    double sec_theta_u = 1/cos_theta_u;

    double sin_psi_u = sin(x8);
    double cos_psi_u = cos(x8);

    // Drone 2 (v)
    double x18=x(18);
    double x19=x(19);
    double x20=x(20);
    double x21=x(21);
    double x22=x(22);
    double x23=x(23);

    double sin_phi_v = sin(x18);
    double cos_phi_v = cos(x18);

    double sin_theta_v = sin(x19);
    double cos_theta_v = cos(x19);
    double tan_theta_v = tan(x19);
    double sec_theta_v = 1/cos_theta_v;

    double sin_psi_v = sin(x20);
    double cos_psi_v = cos(x20);

    // u control
    double u0 = ui_(0);
    double u1 = ui_(1);
    double u2 = ui_(2);
    double u3 = ui_(3);

    // v control
    double v0 = vi_(0);
    double v1 = vi_(1);
    double v2 = vi_(2);
    double v3 = vi_(3);

    //shallow copy
    VectorXd dx(num_states);    // This should be 24 since this is a pursuit situation
    dx.setZero();

    // Drone 1 (u)
    dx(0)=x(3);
    dx(1)=x(4);
    dx(2)=x(5);
    dx(3)=((u0 + v0)*(cos_phi_u*cos_psi_u*sin_theta_u + sin_phi_u*sin_psi_u))/m;
    dx(4)=((u0 + v0)*(-(cos_psi_u*sin_phi_u) + cos_phi_u*sin_theta_u*sin_psi_u))/m;
    dx(5)= -grav + ((u0 + v0)*cos_phi_u*cos_theta_u)/m;
    dx(6)=x9 + x11*cos_phi_u*tan_theta_u + x10*sin_phi_u*tan_theta_u;
    dx(7)=x10*cos_phi_u - x11*sin_phi_u;
    dx(8)=x11*cos_phi_u*sec_theta_u + x10*sec_theta_u*sin_phi_u;
    dx(9)=(u1 + v1 + Iyy*x10*x11 - Izz*x10*x11)/Ixx;
    dx(10)=(u2 + v2 - Ixx*x11*x9 + Izz*x11*x9)/Iyy;
    dx(11)=(u3 + v3 + Ixx*x10*x9 - Iyy*x10*x9)/Izz;

    // Drone 2 (v)
    dx(12)=x(15);
    dx(13)=x(16);
    dx(14)=x(17);
    dx(15)=((u0 + v0)*(cos_phi_v*cos_psi_v*sin_theta_v + sin_phi_v*sin_psi_v))/m;
    dx(16)=((u0 + v0)*(-(cos_psi_v*sin_phi_v) + cos_phi_v*sin_theta_v*sin_psi_v))/m;
    dx(17)= -grav + ((u0 + v0)*cos_phi_v*cos_theta_v)/m;
    dx(18)=x21 + x23*cos_phi_v*tan_theta_v + x22*sin_phi_v*tan_theta_v;
    dx(19)=x22*cos_phi_v - x23*sin_phi_v;
    dx(20)=x23*cos_phi_v*sec_theta_v + x22*sec_theta_v*sin_phi_v;
    dx(21)=(u1 + v1 + Iyy*x22*x23 - Izz*x22*x23)/Ixx;
    dx(22)=(u2 + v2 - Ixx*x23*x21 + Izz*x23*x21)/Iyy;
    dx(23)=(u3 + v3 + Ixx*x22*x21 - Iyy*x22*x21)/Izz;

    dxdt=dx;

}

/**
 forward_propagate_mm calls ode object dynamics_mm at each iteration to propagate
 dynamics given u_traj, v_traj
 @param u_traj_ - a num_controls x 1 VectorXd of stabilizing controls
 @param v_traj - a num_controls x 1 VectorXd of destabilizing controls
 @update x_traj - update x_traj
 */
void PursuitDrones::forward_propagate_mm(vector<VectorXd>& x_traj, const vector<VectorXd>& u_traj, const vector<VectorXd>& v_traj)
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
        stepper.do_step(std::bind(&::PursuitDrones::dynamics_mm, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), x, t, dt);
        t += dt;
        x_traj[i+1] = x; //save x trajectory
        t += dt;
    } //for num_time_steps-1

} //forward_propagate_mm


// TODO: figure out how to make this work for pursuit
void PursuitDrones::feedforward_controls(VectorXd current_state, const deque<VectorXd>& u_traj, const deque<MatrixXd>& K_traj, deque<VectorXd>& x_traj)
{
    // initial condition
    VectorXd x = current_state;
    VectorXd v_zero(4);
    v_zero << 0, 0, 0, 0;

    // Result
    deque<VectorXd> result = x_traj;

    double t=0;
    // main integration loop : propagation over time step
    for (int i = 0; i < Constants::num_time_steps - 1; i++) {
        // update current inputs
        ui_ = u_traj[i] + K_traj[i] * (x - x_traj[i]);
        vi_ = v_zero;

        stepper.do_step(std::bind(&::PursuitDrones::dynamics_mm, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), x, t, dt);
        t += dt;
        result[i] = x; //save x trajectory
        t += dt;
    } //for num_time_steps-1

    // Set x_traj to the new state
    x_traj = result;

} //forward_propagate_mm


/**
 linearize_dynamics_mm obtains the gradient of f (A,B,C) at the given trajectory
 @param x_traj -  a num_states x 1 VectorXd of state values
 @param u_traj_ - a num_controls x 1 VectorXd of stabilizing controls
 @param v_traj - a num_controls x 1 VectorXd of destabilizing controls
 @return A,B,C - gradient Matrix Fx Fu Fv
 */
void PursuitDrones::linearize_dynamics_mm(const vector<VectorXd>& x_traj, const vector<VectorXd>& u_traj, const vector<VectorXd>& v_traj,
                                      vector<MatrixXd>& A,vector<MatrixXd>& B,vector<MatrixXd>& C){

    for (int i=0;i<num_time_steps-1;i++){
        // Drone 1 (u)
        double x6=x_traj[i](6);
        double x7=x_traj[i](7);
        double x8=x_traj[i](8);
        double x9=x_traj[i](9);
        double x10=x_traj[i](10);
        double x11=x_traj[i](11);

        double sin_phi_u = sin(x6);
        double cos_phi_u = cos(x6);

        double sin_theta_u = sin(x7);
        double cos_theta_u = cos(x7);
        double tan_theta_u = tan(x7);
        double sec_theta_u = 1/cos_theta_u;

        double sin_psi_u = sin(x8);
        double cos_psi_u = cos(x8);

        // Drone 2 (v)
        double x18=x_traj[i](18);
        double x19=x_traj[i](19);
        double x20=x_traj[i](20);
        double x21=x_traj[i](21);
        double x22=x_traj[i](22);
        double x23=x_traj[i](23);

        double sin_phi_v = sin(x18);
        double cos_phi_v = cos(x18);

        double sin_theta_v = sin(x19);
        double cos_theta_v = cos(x19);
        double tan_theta_v = tan(x19);
        double sec_theta_v = 1/cos_theta_v;

        double sin_psi_v = sin(x20);
        double cos_psi_v = cos(x20);

        // Control signals
        double u0 = u_traj[i](0);
        double v0 = v_traj[i](0);

        MatrixXd A_sec;
        MatrixXd B_sec;
        MatrixXd C_sec;
        A_sec.setZero(num_states,num_states);
        B_sec.setZero(num_states,num_controls_u);
        C_sec.setZero(num_states,num_controls_v);

        // Drone 1 (u)
        A_sec(0,3)=1;
        A_sec(1,4)=1;
        A_sec(2,5)=1;
        A_sec(3,6)=((u0 + v0)*(-(cos_psi_u*sin_phi_u*sin_theta_u) + cos_phi_u*sin_psi_u))/m;
        A_sec(3,7)=((u0 + v0)*cos_phi_u*cos_theta_u*cos_psi_u)/m;
        A_sec(3,8)=((u0 + v0)*(cos_psi_u*sin_phi_u - cos_phi_u*sin_theta_u*sin_psi_u))/m;
        A_sec(4,6)=-(((u0 + v0)*(cos_phi_u*cos_psi_u + sin_phi_u*sin_theta_u*sin_psi_u))/m);
        A_sec(4,7)=((u0 + v0)*cos_phi_u*cos_theta_u*sin_psi_u)/m;
        A_sec(4,8)=((u0 + v0)*(cos_phi_u*cos_psi_u*sin_theta_u + sin_phi_u*sin_psi_u))/m;
        A_sec(5,6)=-(((u0 + v0)*cos_theta_u*sin_phi_u)/m);
        A_sec(5,7)=-(((u0 + v0)*cos_phi_u*sin_theta_u)/m);
        A_sec(6,6)=(x10*cos_phi_u - x11*sin_phi_u)*tan_theta_u;
        A_sec(6,7)=pow(sec_theta_u,2)*(x11*cos_phi_u + x10*sin_phi_u);
        A_sec(6,9)=1;
        A_sec(6,10)=sin_phi_u*tan_theta_u;
        A_sec(6,11)=cos_phi_u*tan_theta_u;
        A_sec(7,6)=-(x11*cos_phi_u) - x10*sin_phi_u;
        A_sec(7,10)=cos_phi_u;
        A_sec(7,11)=-sin_phi_u;
        A_sec(8,6)=sec_theta_u*(x10*cos_phi_u - x11*sin_phi_u);
        A_sec(8,7)=sec_theta_u*(x11*cos_phi_u + x10*sin_phi_u)*tan_theta_u;
        A_sec(8,10)=sec_theta_u*sin_phi_u;
        A_sec(8,11)=cos_phi_u*sec_theta_u;
        A_sec(9,10)=((Iyy - Izz)*x11)/Ixx;
        A_sec(9,11)=((Iyy - Izz)*x10)/Ixx;
        A_sec(10,9)=((-Ixx + Izz)*x11)/Iyy;
        A_sec(10,11)=((-Ixx + Izz)*x9)/Iyy;
        A_sec(11,9)=((Ixx - Iyy)*x10)/Izz;
        A_sec(11,10)=((Ixx - Iyy)*x9)/Izz;

        B_sec(3,0)=(cos_phi_u*cos_psi_u*sin_theta_u + sin_phi_u*sin_psi_u)/m;
        B_sec(4,0)=(-(cos_psi_u*sin_phi_u) + cos_phi_u*sin_theta_u*sin_psi_u)/m;
        B_sec(5,0)=(cos_phi_u*cos_theta_u)/m;
        B_sec(9,1)=1/Ixx;
        B_sec(10,2)=1/Iyy;
        B_sec(11,3)=1/Izz;

        C_sec(3,0)=(cos_phi_u*cos_psi_u*sin_theta_u + sin_phi_u*sin_psi_u)/m;
        C_sec(4,0)=(-(cos_psi_u*sin_phi_u) + cos_phi_u*sin_theta_u*sin_psi_u)/m;
        C_sec(5,0)=(cos_phi_u*cos_theta_u)/m;
        C_sec(9,1)=1/Ixx;
        C_sec(10,2)=1/Iyy;
        C_sec(11,3)=1/Izz;



        // Drone 2 (v)
        A_sec(12,15)=1;
        A_sec(13,16)=1;
        A_sec(14,17)=1;
        A_sec(15,18)=((u0 + v0)*(-(cos_psi_v*sin_phi_v*sin_theta_v) + cos_phi_v*sin_psi_v))/m;
        A_sec(15,19)=((u0 + v0)*cos_phi_v*cos_theta_v*cos_psi_v)/m;
        A_sec(15,20)=((u0 + v0)*(cos_psi_v*sin_phi_v - cos_phi_v*sin_theta_v*sin_psi_v))/m;
        A_sec(16,18)=-(((u0 + v0)*(cos_phi_v*cos_psi_v + sin_phi_v*sin_theta_v*sin_psi_v))/m);
        A_sec(16,19)=((u0 + v0)*cos_phi_v*cos_theta_v*sin_psi_v)/m;
        A_sec(16,20)=((u0 + v0)*(cos_phi_v*cos_psi_v*sin_theta_v + sin_phi_v*sin_psi_v))/m;
        A_sec(17,18)=-(((u0 + v0)*cos_theta_v*sin_phi_v)/m);
        A_sec(17,19)=-(((u0 + v0)*cos_phi_v*sin_theta_v)/m);
        A_sec(18,18)=(x10*cos_phi_v - x11*sin_phi_v)*tan_theta_v;
        A_sec(18,19)=pow(sec_theta_v,2)*(x11*cos_phi_v + x10*sin_phi_v);
        A_sec(18,21)=1;
        A_sec(18,22)=sin_phi_v*tan_theta_v;
        A_sec(18,23)=cos_phi_v*tan_theta_v;
        A_sec(19,18)=-(x11*cos_phi_v) - x10*sin_phi_v;
        A_sec(19,22)=cos_phi_v;
        A_sec(19,23)=-sin_phi_v;
        A_sec(20,18)=sec_theta_v*(x10*cos_phi_v - x11*sin_phi_v);
        A_sec(20,19)=sec_theta_v*(x11*cos_phi_v + x10*sin_phi_v)*tan_theta_v;
        A_sec(20,22)=sec_theta_v*sin_phi_v;
        A_sec(20,23)=cos_phi_v*sec_theta_v;
        A_sec(21,22)=((Iyy - Izz)*x11)/Ixx;
        A_sec(21,23)=((Iyy - Izz)*x10)/Ixx;
        A_sec(22,21)=((-Ixx + Izz)*x11)/Iyy;
        A_sec(22,23)=((-Ixx + Izz)*x9)/Iyy;
        A_sec(23,21)=((Ixx - Iyy)*x10)/Izz;
        A_sec(23,22)=((Ixx - Iyy)*x9)/Izz;

        B_sec(15,12)=(cos_phi_v*cos_psi_v*sin_theta_v + sin_phi_v*sin_psi_v)/m;
        B_sec(16,12)=(-(cos_psi_v*sin_phi_v) + cos_phi_v*sin_theta_v*sin_psi_v)/m;
        B_sec(17,12)=(cos_phi_v*cos_theta_v)/m;
        B_sec(21,13)=1/Ixx;
        B_sec(22,14)=1/Iyy;
        B_sec(23,15)=1/Izz;

        C_sec(15,12)=(cos_phi_v*cos_psi_v*sin_theta_v + sin_phi_v*sin_psi_v)/m;
        C_sec(16,12)=(-(cos_psi_v*sin_phi_v) + cos_phi_v*sin_theta_v*sin_psi_v)/m;
        C_sec(17,12)=(cos_phi_v*cos_theta_v)/m;
        C_sec(21,13)=1/Ixx;
        C_sec(22,14)=1/Iyy;
        C_sec(23,15)=1/Izz;

        // Apply values to A, B, and C
        A[i]=A_sec;
        B[i]=B_sec;
        C[i]=C_sec;

    } //for (int i)
}// linearize_dynamics
