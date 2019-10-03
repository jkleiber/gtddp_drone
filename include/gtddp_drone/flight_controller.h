#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <math.h>

/**
 * PID CONSTANTS
 */
//Time constant
#define TIME_CONST (double)(0.0)


//X VEL
#define XVEL_KP (double)(0.8)
#define XVEL_KI (double)(0.0)
#define XVEL_KD (double)(0.05)
#define XVEL_LIMIT (double)(2.0)

//Y VEL
#define YVEL_KP (double)(0.8)
#define YVEL_KI (double)(0.0)
#define YVEL_KD (double)(0.05)
#define YVEL_LIMIT (double)(2.0)


//Environment constants
#define GRAVITY (double)(-9.81)  //TODO: should this be positive or negative

class FlightController
{
    public:
        FlightController();
        geometry_msgs::Twist update_state(Eigen::VectorXd cur_state);
        void publish_control(geometry_msgs::Twist ctrl_cmd);
        void reset();

    private:
        //Current state
        Eigen::VectorXd current_state;

        //Current command
        geometry_msgs::Twist cur_cmd;

        //Define the PID controller used to handle flight
        class PIDController
        {
            public:
                PIDController();
                virtual ~PIDController();
                void init(double p, double i, double d, double time_cnst, double limit);

                double gain_p;
                double gain_i;
                double gain_d;
                double time_constant;
                double limit;

                double input;
                double dinput;
                double output;
                double p, i, d;

                double update(double input, double x, double dx, double dt);
                void reset();
        };

        // Struct of all the controllers
        typedef struct controllers_t
        {
            PIDController velocity_x;
            PIDController velocity_y;
            PIDController velocity_z;
        } Controllers;

        Controllers controllers;

        // Save the last pitch and roll commands to be used for PID adjustments
        double pitch_command;
        double roll_command;

        /// \brief save last_time
        ros::Time last_time;

        bool initialized;
};

#endif
