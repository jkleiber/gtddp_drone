#include "gtddp_drone/flight_controller.h"

//TODO: we almost certainly do not need all of these controllers
FlightController::FlightController()
{
    //Initialize PIDs
    //ROLL
    controllers.roll.init(ROLL_KP, ROLL_KI, ROLL_KD, TIME_CONST, ROLL_LIMIT);

    //PITCH
    controllers.pitch.init(PITCH_KP, PITCH_KI, PITCH_KD, TIME_CONST, PITCH_LIMIT);

    //YAW
    //controllers.yaw.init(YAW_KP, YAW_KI, YAW_KD, TIME_CONST, YAW_LIMIT);

    //X VEL
    //controllers.velocity_x.init(XVEL_KP, XVEL_KI, XVEL_KD, TIME_CONST, XVEL_LIMIT);

    //Y VEL
    //controllers.velocity_y.init(YVEL_KP, YVEL_KI, YVEL_KD, TIME_CONST, YVEL_LIMIT);

    //Z VEL
    //controllers.velocity_z.init(ZVEL_KP, ZVEL_KI, ZVEL_KD, TIME_CONST, ZVEL_LIMIT);


    //Save the last time for future integration
    last_time = ros::Time::now();

    //Set the current state as uninitialized
    this->initialized = false;
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


void FlightController::publish_control(geometry_msgs::Twist ctrl_cmd)
{
    this->cur_cmd = ctrl_cmd;
}


geometry_msgs::Twist FlightController::update_state(Eigen::VectorXd cur_state)
{
    //Create a command to publish
    geometry_msgs::Twist pid_cmd;

    if(!initialized)
    {
        this->current_state = cur_state;
        last_time = ros::Time::now();
        initialized = true;
        return pid_cmd;
    }

    //Find the time difference in nanoseconds, then convert to seconds
    double dt = (double)((ros::Time::now() - last_time).toNSec()) / 1000.0 / 1000.0 / 1000.0;
    dt = std::max(dt, 0.001);

    //TODO: use PID for z dot and psi dot?
    //Find acceleration for x and y
    double accel_x = (cur_state(3) - this->current_state(3)) / dt;
    double accel_y = (cur_state(4) - this->current_state(4)) / dt;

    //Calculate the roll and pitch commands
    double pitch_command = this->cur_cmd.linear.x + controllers.velocity_x.update(this->cur_cmd.linear.x, cur_state(3), accel_x, dt);// / GRAVITY;
    double roll_command  = this->cur_cmd.linear.y + controllers.velocity_y.update(this->cur_cmd.linear.y, cur_state(4), accel_y, dt);// / GRAVITY;

    //printf("Pitch commanded: %f\n", pitch_command);

    //Add the PID output to the control command, and then pass through the commanded vertical speed and yaw rate
    pid_cmd.linear.x = pitch_command;
    pid_cmd.linear.y = roll_command;
    pid_cmd.linear.z = this->cur_cmd.linear.z;
    pid_cmd.angular.z = this->cur_cmd.angular.z;

    //Update the current state and timing
    this->current_state = cur_state;
    last_time = ros::Time::now();

    return pid_cmd;
}


/****************************
 * PID Controller Functions
 ****************************/

void FlightController::PIDController::init(double p, double i, double d, double time_cnst, double limit)
{
    this->gain_p = p;
    this->gain_i = i;
    this->gain_d = d;
    this->time_constant = time_cnst;
    this->limit = limit;
}


double FlightController::PIDController::update(double new_input, double x, double dx, double dt)
{
    // limit command
    if (this->limit > 0.0 && fabs(new_input) > this->limit)
    {
        new_input = (new_input < 0 ? -1.0 : 1.0) * this->limit;
    }

    // filter command
    if (dt + time_constant > 0.0)
    {
        this->dinput = (new_input - input) / (dt + time_constant);
        this->input  = (dt * new_input + time_constant * input) / (dt + time_constant);
    }

    // update proportional, differential and integral errors
    this->p = input - x;
    this->d = dinput - dx;
    this->i = i + dt * p;

    // update control output
    output = gain_p * p + gain_d * d + gain_i * i;
    return output;
}

void FlightController::PIDController::reset()
{
    input = dinput = 0;
    p = i = d = output = 0;
}


FlightController::PIDController::PIDController()
{

}

FlightController::PIDController::~PIDController()
{

}
