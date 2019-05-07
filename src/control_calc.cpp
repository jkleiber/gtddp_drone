#include "gtddp_drone/control_calc.h"

/**
 * 
 */
ControlCalculator::ControlCalculator()
{
    //Initialize current state size
    this->cur_state.resize(Constants::num_states);

    //Reset flags
    this->cur_state_init = false;
    this->traj_init = false;

    //Set timestep to t0
    this->timestep = -1;

    //Set the status to idle
    ctrl_status.status = gtddp_drone_msgs::Status::IDLE;

    //Initialize logging
    this->logging_init();

    //Set the beginning time
    this->begin_time = ros::Time::now();
}


/**
 * 
 */
ControlCalculator::ControlCalculator(ros::Publisher ctrl_sig_pub, ros::Publisher status_pub, int sim_status)
{
    //Control signal to publish to AR Drone
    this->control_signal_pub = ctrl_sig_pub;

    //For publishing the control status
    this->status_pub = status_pub;

    //Initialize current state size
    this->cur_state.resize(Constants::num_states);

    //Reset flags
    this->cur_state_init = false;
    this->traj_init = false;

    //Set timestep to t0
    this->timestep = -1;

    //Set the status to idle
    ctrl_status.status = gtddp_drone_msgs::Status::IDLE;
    this->status_pub.publish(this->ctrl_status);

    //Set simulation status
    this->is_sim = sim_status;

    //Initialize logging
    this->logging_init();

    //Set the beginning time
    this->begin_time = ros::Time::now();
}



void ControlCalculator::logging_init()
{
    //Declare local variables
    std::string filename;
    const char *home_dir;
    std::stringstream ss;

    //Find the user's home directory
    if ((home_dir = getenv("HOME")) == NULL) 
    {
        home_dir = getpwuid(getuid())->pw_dir;
    }

    //If the home directory checks failed, don't log anything
    if(home_dir == NULL)
    {
        //Print a lot of errors so somebody notices
        printf("CONTROL LOGGING FAILED TO REGISTER!!!!!!!!\n");
        printf("CONTROL LOGGING FAILED TO REGISTER!!!!!!!!\n");
        printf("CONTROL LOGGING FAILED TO REGISTER!!!!!!!!\n");
        printf("CONTROL LOGGING FAILED TO REGISTER!!!!!!!!\n");
        printf("CONTROL LOGGING FAILED TO REGISTER!!!!!!!!\n");
        printf("CONTROL LOGGING FAILED TO REGISTER!!!!!!!!\n");
        return;
    }

    //Convert the home directory into a string
    std::string filepath(home_dir);

    //Add a / to the end of the home directory if needed
    if(home_dir[strlen(home_dir)-1] != '/')
    {
        filepath += "/";
    }

    //Get the timestamp
    auto cur_stamp = std::chrono::system_clock::now();
    std::time_t timestamp = std::chrono::system_clock::to_time_t(cur_stamp);

    //Form the file name
    ss << filepath << timestamp << "_gt_data" << ".csv";
    ss >> filename;
    std::cout << "Logging ground_truth_data to " << filename << std::endl; 

    //Open the ground truth log file
    this->ground_truth_data.open(filename);

    //Reset the filename for the command data log file
    filename = "";
    ss.clear();
    ss << filepath << timestamp << "_cmd_data" << ".csv";
    ss >> filename;
    std::cout << "Logging control commands to " << filename << std::endl;

    //Open the command log file
    this->command_data.open(filename);
}


/**
 * 
 */
void ControlCalculator::recalculate_control_callback(const ros::TimerEvent& time_event)
{
    //Get the current time
    double cur_time = (ros::Time::now() - begin_time).toSec();

    //Only output to the control topic if the localization has happened and the trajectory has been built
    //Also only output if the timestep is in bounds
    if(this->cur_state_init && this->traj_init
    && timestep >= 0 && timestep < this->x_traj.size())
    {
        //Set the status to flying
        ctrl_status.status = gtddp_drone_msgs::Status::FLYING;
        this->status_pub.publish(this->ctrl_status);
/*
        printf("x: [");
        for(int i = 0; i < Constants::num_states; ++i)
        {
            printf("%f ", this->x_traj[timestep](i));
        }
        printf("]\n");*/
        
        /* Form the control message */
        //Simulation takes velocity
        if(this->is_sim)
        {
            //X velocity (move forward) (x dot)
            this->ctrl_command.linear.x = this->x_traj[timestep](3);// / MAX_FWD_VEL;

            //Y velocity (move side to side) (y dot)
            this->ctrl_command.linear.y = this->x_traj[timestep](4);// / MAX_SIDE_VEL;
        }
        //Real drones take pitch and negative roll
        else
        {
            //Pitch (move forward) (phi)
            this->ctrl_command.linear.x = this->x_traj[timestep](6);

            //Roll (move sideways) (theta)
            this->ctrl_command.linear.y = -this->x_traj[timestep](7);
        }
        
        //Yaw rate (how fast to spin) (r)
        //this->ctrl_command.angular.z = this->angleWrap(this->x_traj[timestep](11));// / MAX_YAW_RATE;
        this->ctrl_command.angular.z = 0;

        //Vertical speed (how fast to move upward) (z dot)
        this->ctrl_command.linear.z = this->x_traj[timestep](5);// / MAX_VERTICAL_VEL;

        //Increment the timestep
        this->timestep++;

        ctrl_timer.setPeriod(ros::Duration(0.001));
    }
    //TODO: will removing this code crash a real-life drone? (probably) 
    //TODO: Currently excluded for simulations only (due to accumulation of error)
    else if((this->cur_state_init && this->traj_init) || !this->is_sim)
    {
        //Set the status to idle
        ctrl_status.status = gtddp_drone_msgs::Status::IDLE;
        this->status_pub.publish(this->ctrl_status);

        //Hover until next command
        this->ctrl_command.linear.x = 0;
        this->ctrl_command.linear.y = 0;
        this->ctrl_command.linear.z = 0;
        this->ctrl_command.angular.x = 0;
        this->ctrl_command.angular.y = 0;
        this->ctrl_command.angular.z = 0;
        //TODO: is a separate hover controller necessary for a real drone?

        //Slow down updates in the simulator because this will make the hover less bad
        if(is_sim)
        {
            ctrl_timer.setPeriod(ros::Duration(1.0));
        }
    }

    //Log the commands
    std::string data_str = std::to_string(cur_time) + "," + std::to_string(ctrl_command.linear.x) + "," + std::to_string(ctrl_command.linear.y) + "," + std::to_string(ctrl_command.linear.z) + "," + std::to_string(ctrl_command.angular.z) + "\n";
    this->command_data << data_str;

    //If this is a simulation, publish the control command directly
    if(this->is_sim)
    {
        this->control_signal_pub.publish(this->ctrl_command);
    }
    //Otherwise, publish data after going through the flight controller
    else
    {
        //FIXME: PID controller commands a really weird pitch
        //flight_controller.publish_control(this->ctrl_command);

        this->control_signal_pub.publish(this->ctrl_command);
    }
}


/**
 * 
 */
void ControlCalculator::state_estimate_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
    //Position
    this->cur_state(0) = odom->pose.pose.position.x;
    this->cur_state(1) = odom->pose.pose.position.y;
    this->cur_state(2) = odom->pose.pose.position.z;

    //Orientation
    tf::Pose pose;
    tf::poseMsgToTF(odom->pose.pose, pose);
    tf::Matrix3x3 mat(pose.getRotation());

    //Convert the quaternion to euler angles of yaw, pitch, and roll
    //This should be in radians
    mat.getEulerYPR(this->cur_state(8), this->cur_state(7), this->cur_state(6));
    //mat.getEulerYPR(this->cur_state(5), this->cur_state(4), this->cur_state(3));

    //Linear velocity
    this->cur_state(3) = odom->twist.twist.linear.x;
    this->cur_state(4) = odom->twist.twist.linear.y;
    this->cur_state(5) = odom->twist.twist.linear.z;
    //this->cur_state(6) = odom->twist.twist.linear.x;
    //this->cur_state(7) = odom->twist.twist.linear.y;
    //this->cur_state(8) = odom->twist.twist.linear.z;

    //Angular velocity
    this->cur_state(9) = odom->twist.twist.angular.x;
    this->cur_state(10) = odom->twist.twist.angular.y;
    this->cur_state(11) = odom->twist.twist.angular.z;

    //printf("CS: [");
    //for(int i = 0; i < Constants::num_states; ++i)
    //{
    //    printf("%f ", this->cur_state(i));
    //}
    //printf("]\n");
    double cur_time = (ros::Time::now() - begin_time).toSec();

    //Output data to the logging file
    std::string data_str = std::to_string(cur_time) + "," + std::to_string(cur_state(0)) + "," + std::to_string(cur_state(1)) + "," + std::to_string(cur_state(2)) + "\n";
    //std::cout << data_str << std::endl;
    this->ground_truth_data << data_str;

    //Set the current state as initialized
    this->cur_state_init = true;

    //FIXME: drone flies forward, PID control doesn't work right
    /*
    //If this is real-life, update the output
    if(this->is_sim == false)
    {
        this->control_signal_pub.publish(flight_controller.update_state(this->cur_state));
    }*/
}



//TODO: use tum_ardrone or not?
/**
 * 
 */
/*
void ControlCalculator::state_estimate_callback(const tum_ardrone::filter_state::ConstPtr& estimate_event)
{
    //Update the current state from the tum_ardrone
    this->cur_state(0) = estimate_event->x;
    this->cur_state(1) = estimate_event->y;
    this->cur_state(2) = estimate_event->z;

    this->cur_state(3) = estimate_event->dx;
    this->cur_state(4) = estimate_event->dy;
    this->cur_state(5) = estimate_event->dz;

    this->cur_state(6) = estimate_event->roll;
    this->cur_state(7) = estimate_event->pitch;
    this->cur_state(8) = estimate_event->yaw;
    
    this->cur_state(9) = estimate_event->droll;
    this->cur_state(10) = estimate_event->dpitch;
    this->cur_state(11) = estimate_event->dyaw;

    //TODO: Review dynamics
    //this->cur_state(3) = estimate_event->roll;
    //this->cur_state(4) = estimate_event->pitch;
    //this->cur_state(5) = estimate_event->yaw;
    //this->cur_state(6) = estimate_event->dx;
    //this->cur_state(7) = estimate_event->dy;
    //this->cur_state(8) = estimate_event->dz;

    //Set the current state as initialized
    this->cur_state_init = true;
}
*/

/**
 * 
 */
void ControlCalculator::trajectory_callback(const gtddp_drone_msgs::Trajectory::ConstPtr& traj_msg)
{
    //Declare local variables
    int i;          //iteration variable
    int num_events; //number of events in the message
    int t;          //time iteration variable
    int r, c;       //rows and columns of gain matrix

    //Set the status to flying
    ctrl_status.status = gtddp_drone_msgs::Status::FLYING;
    this->status_pub.publish(this->ctrl_status);

    //Get the components of the message
    const std::vector<gtddp_drone_msgs::state_data> &x_data = traj_msg->x_traj;
    const std::vector<gtddp_drone_msgs::ctrl_data> &u_data = traj_msg->u_traj;
    const std::vector<gtddp_drone_msgs::gain_data> &K_data = traj_msg->K_traj;

    //Find the number of events
    num_events = x_data.size();

    //Clear the trajectory vectors
    this->x_traj.clear();
    this->u_traj.clear();
    this->K_traj.clear();

    //Create temporary vectors for the trajectory data
    Eigen::VectorXd x_traj_dat(Constants::num_states);
    Eigen::VectorXd u_traj_dat(Constants::num_controls_u);
    Eigen::MatrixXd K_traj_dat(Constants::num_controls_u, Constants::num_states);

    //Parse the message to get the trajectory
    for(t = 0; t < num_events; ++t)
    {
        //x_traj
        for(i = 0; i < Constants::num_states; ++i)
        {
            x_traj_dat(i) = x_data[t].states[i];
        }

        //u_traj
        for(i = 0; i < Constants::num_controls_u; ++i)
        {
            u_traj_dat(i) = u_data[t].ctrl[i];
        }

        //K_traj
        //Go through rows and columns of the message
        for(r = 0; r < Constants::num_controls_u; ++r)
        {
            for(c = 0; c < Constants::num_states; ++c)
            {
                K_traj_dat(r,c) = K_data[t].gain_mat[r].gain_list[c];
            }
        }

        //Push the new data into storage
        this->x_traj.push_back(x_traj_dat);
        this->u_traj.push_back(u_traj_dat);
        this->K_traj.push_back(K_traj_dat);
    }

    //Forward propagate controls to find the correct output based on the starting position
    //This serves to get the drone back on track when it strays due to hover, wind, etc.
    quadrotor.feedforward_controls(this->cur_state, this->u_traj, this->K_traj, this->x_traj);

    //Reset the timestep variable to t0
    timestep = 0;

    //The trajectory is now initialized!
    this->traj_init = true;
}



void ControlCalculator::set_timer(ros::Timer& timer)
{
    this->ctrl_timer = timer;
}


ControlCalculator::~ControlCalculator()
{
    this->ground_truth_data.close();
    this->command_data.close();
}

double ControlCalculator::angleWrap(double angle)
{
    double wrapped_angle;

    //Get the remainder of (angle shifted by pi) / 2pi to force the angle between -2pi and 2pi
    wrapped_angle = fmod(angle + Constants::pi, 2 * Constants::pi);

    //If the remainder is negative, add 2pi to make it positive
    if(wrapped_angle < 0)
    {
        wrapped_angle += (2 * Constants::pi);
    }

    //Return the angle wrapped to -pi to pi
    return (wrapped_angle - Constants::pi);
}
