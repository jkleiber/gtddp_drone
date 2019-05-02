#include "gtddp_drone/flight_controller.h"


FlightController::FlightController()
{
    //Initialize PIDs
    //ROLL
    controllers.roll.init(ROLL_KP, ROLL_KI, ROLL_KD, TIME_CONST, ROLL_LIMIT);

    //PITCH
    controllers.pitch.init(PITCH_KP, PITCH_KI, PITCH_KD, TIME_CONST, PITCH_LIMIT);

    //YAW
    controllers.yaw.init(YAW_KP, YAW_KI, YAW_KD, TIME_CONST, YAW_LIMIT);

    //X VEL
    controllers.velocity_x.init(XVEL_KP, XVEL_KI, XVEL_KD, TIME_CONST, XVEL_LIMIT);

    //Y VEL
    controllers.velocity_y.init(YVEL_KP, YVEL_KI, YVEL_KD, TIME_CONST, YVEL_LIMIT);

    //Z VEL
    controllers.velocity_z.init(ZVEL_KP, ZVEL_KI, ZVEL_KD, TIME_CONST, ZVEL_LIMIT);


    //Save the last time for future integration
    last_time = ros::Time::now();
}


void FlightController::reset()
{
    //Reset the last time
    last_time = ros::Time::now();

    //Reset all of the PID controllers
    controllers.pitch.reset();
    controllers.roll.reset();
    controllers.yaw.reset();
    controllers.velocity_x.reset();
    controllers.velocity_y.reset();
    controllers.velocity_z.reset();
}


void FlightController::PIDController::init(double p, double i, double d, double time_cnst, double limit)
{
    this->gain_p = p;
    this->gain_i = i;
    this->gain_d = d;
    this->time_constant = time_cnst;
    this->limit = limit;
}


double FlightController::PIDController::update(double input, double x, double dx, double dt)
{

}