#include "gtddp_drone/control_calc.h"

/**
 *
 */
ControlCalculator::ControlCalculator() : drone_traj()
{
    //Initialize current state size
    this->cur_state.resize(Constants::num_states);

    //Reset flags
    this->cur_state_init = false;
    this->cur_state_init_2 = false;
    this->traj_init = false;

    //Initialize logging
    this->logging_init();

    // Track timestep
    this->timestep = 0;

    // Assume Quadrotor controller
    // TODO: move to new design pattern
    this->drone = new Quadrotor();
    this->is_pursuit = false;
}


ControlCalculator::ControlCalculator(int sim_status) : drone_traj()
{
    //Initialize current state size
    this->cur_state.resize(Constants::num_states);

    //Reset flags
    this->cur_state_init = false;
    this->cur_state_init_2 = false;
    this->traj_init = false;
    this->is_pursuit = false;

    //Set simulation status
    this->is_sim = sim_status;

    //Initialize logging
    this->logging_init();

    // Track timestep
    this->timestep = 0;

    // Assume Quadrotor controller
    // TODO: move to new design pattern
    this->drone = new Quadrotor();
}



/**
 *
 */
ControlCalculator::ControlCalculator(ros::Publisher ctrl_sig_pub, int sim_status) : drone_traj()
{
    //Control signal to publish to AR Drone
    this->control_signal_pub = ctrl_sig_pub;

    //Initialize current state size
    this->cur_state.resize(Constants::num_states);

    //Reset flags
    this->cur_state_init = false;
    this->cur_state_init_2 = false;
    this->traj_init = false;
    this->is_pursuit = false;

    //Set simulation status
    this->is_sim = sim_status;

    //Initialize logging
    this->logging_init();

    //Set the beginning time
    this->begin_time = ros::Time::now();

    // Track timestep
    this->timestep = 0;

    // Assume Quadrotor controller
    // TODO: move to new design pattern
    this->drone = new Quadrotor();
}



ControlCalculator::ControlCalculator(ros::Publisher ctrl_sig_pub,
                                     int sim_status,
                                     bool is_pursuit,
                                     ros::Publisher ctrl_sig_pub_2) : drone_traj()
{
    //Control signal to publish to AR Drone 1
    this->control_signal_pub = ctrl_sig_pub;

    // If this is pursuit, set a second control signal publisher
    // TODO: move to new design pattern
    this->is_pursuit = is_pursuit;
    if(is_pursuit)
    {
        this->control_signal_pub_2 = ctrl_sig_pub_2;
        this->drone = new PursuitDrones();
    }
    else
    {
        this->drone = new Quadrotor();
    }

    //Initialize current state size
    this->cur_state.resize(Constants::num_states);

    //Reset flags
    this->cur_state_init = false;
    this->cur_state_init_2 = false;
    this->traj_init = false;

    //Set simulation status
    this->is_sim = sim_status;

    //Initialize logging
    this->logging_init();

    //Set the beginning time
    this->begin_time = ros::Time::now();

    // Track timestep
    this->timestep = 0;
}


void ControlCalculator::control_init()
{
    //Set the beginning time
    this->begin_time = ros::Time::now();
}


// TODO: allow filepath prefix specification
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

    // If this is a pursuit-evasion situation, then add the pursuit tag
    if(this->is_pursuit)
    {
        filepath += "pursuit_";
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
    // Get the current time
    double cur_time = (ros::Time::now() - begin_time).toSec();

    // Send hover command
    bool hover = false;

    if(Constants::debug_mode)
    {
        if (!this->is_pursuit)
        {
            printf("VICON Connected: %d, Trajectory Initialized: %d, Trajectory Deque Empty?: %d\n",
                    this->cur_state_init, this->traj_init, this->drone_traj.x_traj.empty());
        }
        else
        {
            printf("VICON Connected ardrone1: %d, VICON Connected ardrone2: %d, Trajectory Initialized: %d, Trajectory Deque Empty?: %d\n",
                    this->cur_state_init, this->cur_state_init_2, this->traj_init, this->drone_traj.x_traj.empty());
        }
    }

    // Only output to the control topic if the localization has happened and the trajectory has been built
    if(this->cur_state_init
    && (!this->is_pursuit || this->cur_state_init_2)
    && this->traj_init
    && !this->drone_traj.x_traj.empty())
    {
        // Only predict the next set of timesteps once per interval
        if(this->timestep == 0 && this->drone_traj.x_traj.size() >= Constants::num_time_steps)
        {
            // printf("%f <- %f with ctrl: %f\n", this->drone_traj.x_traj.front()(2), this->cur_state(2), this->drone_traj.x_traj.front()(5));
            //Forward propagate controls to find the correct output based on the starting position
            //This serves to get the drone back on track when it strays due to hover, wind, etc.
            drone->feedforward_controls(this->cur_state, this->drone_traj);
            ++this->timestep;
        }
        // Increment the timestep and reset if needed
        else
        {
            ++this->timestep;
            if(this->timestep > Constants::num_time_steps)
            {
                this->timestep = 0;
            }
        }

        // printf("x: {{");
        // for(int i = 0; i < Constants::num_states; ++i)
        // {
        //     printf("%f ", this->drone_traj.x_traj.front()(i));
        // }
        // printf("}}\n");



        /* Form the control message */
        // Simulation does PID for us, so clamp the output
        if(this->is_sim)
        {
            // X velocity (move forward) (x dot)
            this->ctrl_command.linear.x = this->clamp(this->drone_traj.x_traj.front()(3), -1.0, 1.0);

            // Y velocity (move side to side) (y dot)
            this->ctrl_command.linear.y = this->clamp(this->drone_traj.x_traj.front()(4), -1.0, 1.0);

            // Pursuit X and Y velocity
            if(this->is_pursuit)
            {
                this->ctrl_command_2.linear.x = this->clamp(this->drone_traj.x_traj.front()(15), -1.0, 1.0);
                this->ctrl_command_2.linear.y = this->clamp(this->drone_traj.x_traj.front()(16), -1.0, 1.0);
            }
        }
        // For real life, we implement our own PID controller
        // which clamps the output, so pass in raw values
        else
        {
            // X velocity (move forward) (x dot)
            this->ctrl_command.linear.x = this->drone_traj.x_traj.front()(3);

            // Y velocity (move side to side) (y dot)
            this->ctrl_command.linear.y = this->drone_traj.x_traj.front()(4);

            // Pursuit X and Y velocity
            if(this->is_pursuit)
            {
                this->ctrl_command_2.linear.x = this->clamp(this->drone_traj.x_traj.front()(15), -1.0, 1.0);
                this->ctrl_command_2.linear.y = this->clamp(this->drone_traj.x_traj.front()(16), -1.0, 1.0);
            }
        }

        // Yaw rate (how fast to spin) (r)
        // HACK: set this to 0 for now because it causes flight instability
        this->ctrl_command.angular.z = 0;


        // Vertical speed (how fast to move upward) (z dot)
        this->ctrl_command.linear.z = this->clamp(this->drone_traj.x_traj.front()(5), -1.0, 1.0);

        // Evader control if needed
        if (this->is_pursuit)
        {
            this->ctrl_command_2.angular.z = 0;
            this->ctrl_command_2.linear.z = this->clamp(this->drone_traj.x_traj.front()(17), -1.0, 1.0);
        }

        // Pop the front element off the deques
        this->drone_traj.x_traj.pop_front();
        this->drone_traj.u_traj.pop_front();
        this->drone_traj.v_traj.pop_front();
        this->drone_traj.Ku_traj.pop_front();
        this->drone_traj.Kv_traj.pop_front();

        ctrl_timer.setPeriod(ros::Duration(0.001));
    }
    // Hover if the state and/or trajectory do not exist.
    // Also, do not hover on the first optimization in a simulated environment
    else if((this->cur_state_init && this->traj_init)
         || !this->is_sim
         || this->drone_traj.x_traj.empty())
    {
        // Hover until next command
        hover = true;
        this->ctrl_command.linear.x = 0;
        this->ctrl_command.linear.y = 0;
        this->ctrl_command.linear.z = 0;
        this->ctrl_command.angular.x = 0;
        this->ctrl_command.angular.y = 0;
        this->ctrl_command.angular.z = 0;

        // If this is pursuit, we have two hovers
        if(this->is_pursuit)
        {
            // Hover until next command
            this->ctrl_command_2.linear.x = 0;
            this->ctrl_command_2.linear.y = 0;
            this->ctrl_command_2.linear.z = 0;
            this->ctrl_command_2.angular.x = 0;
            this->ctrl_command_2.angular.y = 0;
            this->ctrl_command_2.angular.z = 0;
        }

        // Slow down updates in the simulator because this will make the hover less bad
        if(is_sim)
        {
            ctrl_timer.setPeriod(ros::Duration(1.0));
        }
        else
        {
            ctrl_timer.setPeriod(ros::Duration(0.1));
        }
    }

    // If this is a simulation, publish the control command directly
    if(this->is_sim)
    {
        this->control_signal_pub.publish(this->ctrl_command);

        if(this->is_pursuit)
        {
            this->control_signal_pub_2.publish(this->ctrl_command_2);
        }
    }
    // Otherwise, publish data after going through the flight controller
    else if (!hover)
    {
        // Publish the control signal
        flight_controller.publish_control(this->ctrl_command);

        // Publish the evader control signal if needed
        if(this->is_pursuit)
        {
            flight_controller_2.publish_control(this->ctrl_command_2);
        }

        // Control for drones
        this->ctrl_command = flight_controller.update_state(this->cur_state);
        this->ctrl_command.linear.x = this->clamp(this->ctrl_command.linear.x, -1.0, 1.0);
        this->ctrl_command.linear.y = this->clamp(this->ctrl_command.linear.y, -1.0, 1.0);

        // Evader control
        if(this->is_pursuit)
        {
            this->ctrl_command_2 = flight_controller_2.update_state(this->cur_state.tail(Constants::num_states / 2));
            this->ctrl_command_2.linear.x = this->clamp(this->ctrl_command_2.linear.x, -1.0, 1.0);
            this->ctrl_command_2.linear.y = this->clamp(this->ctrl_command_2.linear.y, -1.0, 1.0);

            // Publish evader control
            this->control_signal_pub_2.publish(this->ctrl_command_2);
        }

        // Default drone control publish
        this->control_signal_pub.publish(this->ctrl_command);
    }
    // Hover on the real drone
    else
    {
        this->control_signal_pub.publish(this->ctrl_command);

        if(this->is_pursuit)
        {
            this->control_signal_pub_2.publish(this->ctrl_command_2);
        }
    }

    // Log the commands
    std::string data_str =  std::to_string(cur_time) + "," +
                            std::to_string(ctrl_command.linear.x) + "," +
                            std::to_string(ctrl_command.linear.y) + "," +
                            std::to_string(ctrl_command.linear.z) + "," +
                            std::to_string(ctrl_command.angular.z);

    // Add more commands if we are controlling 2 drones
    if(this->is_pursuit)
    {
        data_str += "," + std::to_string(ctrl_command_2.linear.x) + "," +
                    std::to_string(ctrl_command_2.linear.y) + "," +
                    std::to_string(ctrl_command_2.linear.z) + "," +
                    std::to_string(ctrl_command_2.angular.z) + "\n";
    }
    else
    {
        data_str += "\n";
    }

    this->command_data << data_str << std::flush;
}


void ControlCalculator::set_system(Drone *drone)
{
    this->drone = drone;
}


void ControlCalculator::feedback_controller(Eigen::VectorXd current_state)
{
    if(drone_traj.x_traj.size() >= Constants::num_time_steps)
    {
        drone->feedforward_controls(current_state, this->drone_traj);
    }
}


/**
 * @brief Indicates if the trajectory is available to be used for controlling the system
 *
 * @return true The optimized trajectory has been received
 * @return false No trajectory points are in the deque
 */
bool ControlCalculator::is_traj_available()
{
    return !this->drone_traj.x_traj.empty();
}


TrajectoryPoint ControlCalculator::pop_traj_point()
{
    TrajectoryPoint traj_pt;

    // Load the front element into the trajectory point
    traj_pt.x = this->drone_traj.x_traj.front();
    traj_pt.u = this->drone_traj.u_traj.front();
    traj_pt.v = this->drone_traj.v_traj.front();
    traj_pt.Ku = this->drone_traj.Ku_traj.front();
    traj_pt.Kv = this->drone_traj.Kv_traj.front();

    // Pop the front element off the deques
    this->drone_traj.x_traj.pop_front();
    this->drone_traj.u_traj.pop_front();
    this->drone_traj.v_traj.pop_front();
    this->drone_traj.Ku_traj.pop_front();
    this->drone_traj.Kv_traj.pop_front();

    return traj_pt;
}


void ControlCalculator::open_loop_control(const ros::TimerEvent& time_event)
{
    // Get the current time
    double cur_time = (ros::Time::now() - begin_time).toSec();

    // Only output to the control topic if the localization has happened and the trajectory has been built
    if(this->cur_state_init && this->traj_init
    && !this->drone_traj.u_traj.empty())
    {
        // Directly use the controls given by the optimizer
        this->ctrl_command.linear.x = this->drone_traj.u_traj.front()(0);
        this->ctrl_command.linear.y = this->drone_traj.u_traj.front()(1);
        this->ctrl_command.linear.z = this->drone_traj.u_traj.front()(2);
        this->ctrl_command.angular.z = 0;//this->drone_traj.u_traj.front()(3);

        // Pop the front element off the deque
        this->drone_traj.u_traj.pop_front();

        // Publish the control signal
        flight_controller.publish_control(this->ctrl_command);

        // Convert flight control forces into AR.Drone control signal.
        this->ctrl_command = flight_controller.update_state(this->cur_state);
        this->ctrl_command.linear.x = this->clamp(this->ctrl_command.linear.x, -1.0, 1.0);
        this->ctrl_command.linear.y = this->clamp(this->ctrl_command.linear.y, -1.0, 1.0);

        this->control_signal_pub.publish(this->ctrl_command);
    }
    else
    {
        // Hover until next command
        this->ctrl_command.linear.x = 0;
        this->ctrl_command.linear.y = 0;
        this->ctrl_command.linear.z = 0;
        this->ctrl_command.angular.x = 0;
        this->ctrl_command.angular.y = 0;
        this->ctrl_command.angular.z = 0;

        // Publish the flight control
        this->control_signal_pub.publish(this->ctrl_command);
    }

    // Log the commands
    std::string data_str = std::to_string(cur_time) + "," + std::to_string(ctrl_command.linear.x) + "," + std::to_string(ctrl_command.linear.y) + "," + std::to_string(ctrl_command.linear.z) + "," + std::to_string(ctrl_command.angular.z) + "\n";
    this->command_data << data_str << std::flush;
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

    //Angular velocity
    this->cur_state(9) = odom->twist.twist.angular.x;
    this->cur_state(10) = odom->twist.twist.angular.y;
    this->cur_state(11) = odom->twist.twist.angular.z;


    double cur_time = (ros::Time::now() - begin_time).toSec();

    //Output data to the logging file
    std::string data_str = std::to_string(cur_time) + ",";
    for(int i = 0; i < Constants::num_states; ++i)
    {
        data_str += std::to_string(cur_state(i));

        if(i < Constants::num_states - 1)
        {
            data_str += ",";
        }
    }
    data_str += "\n";

    this->ground_truth_data << data_str << std::flush;

    //Set the current state as initialized
    this->cur_state_init = true;
}


/**
 *
 */
void ControlCalculator::state_estimate_callback_2(const nav_msgs::Odometry::ConstPtr& odom)
{
    //Position
    this->cur_state(12) = odom->pose.pose.position.x;
    this->cur_state(13) = odom->pose.pose.position.y;
    this->cur_state(14) = odom->pose.pose.position.z;

    //Orientation
    tf::Pose pose;
    tf::poseMsgToTF(odom->pose.pose, pose);
    tf::Matrix3x3 mat(pose.getRotation());

    //Convert the quaternion to euler angles of yaw, pitch, and roll
    //This should be in radians
    mat.getEulerYPR(this->cur_state(20), this->cur_state(19), this->cur_state(18));

    //Linear velocity
    this->cur_state(15) = odom->twist.twist.linear.x;
    this->cur_state(16) = odom->twist.twist.linear.y;
    this->cur_state(17) = odom->twist.twist.linear.z;

    //Angular velocity
    this->cur_state(21) = odom->twist.twist.angular.x;
    this->cur_state(22) = odom->twist.twist.angular.y;
    this->cur_state(23) = odom->twist.twist.angular.z;


    double cur_time = (ros::Time::now() - begin_time).toSec();

    //Output data to the logging file
    std::string data_str = std::to_string(cur_time) + ",";
    for(int i = 0; i < Constants::num_states; ++i)
    {
        data_str += std::to_string(cur_state(i));

        if(i < Constants::num_states - 1)
        {
            data_str += ",";
        }
    }
    data_str += "\n";

    this->ground_truth_data << data_str << std::flush;

    //Set the current state as initialized
    this->cur_state_init_2 = true;
}


void ControlCalculator::log_ground_truth(Eigen::VectorXd x_data)
{
    double cur_time = (ros::Time::now() - begin_time).toSec();

    //Output data to the logging file
    std::string data_str = std::to_string(cur_time) + ",";
    for(int i = 0; i < Constants::num_states; ++i)
    {
        data_str += std::to_string(x_data(i));

        if(i < Constants::num_states - 1)
        {
            data_str += ",";
        }
    }
    data_str += "\n";

    this->ground_truth_data << data_str << std::flush;
}

void ControlCalculator::log_control(Eigen::VectorXd ctrl_data)
{
    double cur_time = (ros::Time::now() - begin_time).toSec();

    //Output data to the logging file
    std::string data_str = std::to_string(cur_time) + ",";
    for(int i = 0; i < Constants::num_controls_u; ++i)
    {
        data_str += std::to_string(ctrl_data(i));

        if(i < Constants::num_controls_u - 1)
        {
            data_str += ",";
        }
    }
    data_str += "\n";

    this->command_data << data_str << std::flush;
}


/**
 *
 */
void ControlCalculator::trajectory_callback(const gtddp_drone_msgs::Trajectory::ConstPtr& traj_msg)
{
    //Declare local variables
    int i;          //iteration variable
    int t;          //time iteration variable
    int r, c;       //rows and columns of gain matrix

    //Get the components of the message
    const std::vector<gtddp_drone_msgs::state_data> &x_data = traj_msg->x_traj;
    const std::vector<gtddp_drone_msgs::ctrl_data> &u_data = traj_msg->u_traj;
    const std::vector<gtddp_drone_msgs::ctrl_data> &v_data = traj_msg->v_traj;
    const std::vector<gtddp_drone_msgs::gain_data> &Ku_data = traj_msg->Ku_traj;
    const std::vector<gtddp_drone_msgs::gain_data> &Kv_data = traj_msg->Kv_traj;


    //Create temporary vectors for the trajectory data
    Eigen::VectorXd x_traj_dat(Constants::num_states);
    Eigen::VectorXd u_traj_dat(Constants::num_controls_u);
    Eigen::VectorXd v_traj_dat(Constants::num_controls_v);
    Eigen::MatrixXd Ku_traj_dat(Constants::num_controls_u, Constants::num_states);
    Eigen::MatrixXd Kv_traj_dat(Constants::num_controls_v, Constants::num_states);

    //Parse the message to get the trajectory
    for(t = 0; t < x_data.size(); ++t)
    {
        //x_traj
        for(i = 0; i < Constants::num_states; ++i)
        {
            x_traj_dat(i) = x_data[t].state[i];
        }

        //Push the new data into storage
        this->drone_traj.x_traj.push_back(x_traj_dat);
    }

    for(t = 0; t < u_data.size(); ++t)
    {
        //u_traj
        for(i = 0; i < Constants::num_controls_u; ++i)
        {
            u_traj_dat(i) = u_data[t].ctrl[i];
        }

        this->drone_traj.u_traj.push_back(u_traj_dat);
    }

    for(t = 0; t < v_data.size(); ++t)
    {
        //v_traj
        for(i = 0; i < Constants::num_controls_v; ++i)
        {
            v_traj_dat(i) = v_data[t].ctrl[i];
        }

        this->drone_traj.v_traj.push_back(v_traj_dat);
    }

    for(t = 0; t < Ku_data.size(); ++t)
    {
        //Ku_traj
        //Go through rows and columns of the message
        for(r = 0; r < Constants::num_controls_u; ++r)
        {
            for(c = 0; c < Constants::num_states; ++c)
            {
                Ku_traj_dat(r,c) = Ku_data[t].gain_mat[r].gain_list[c];
            }
        }

        this->drone_traj.Ku_traj.push_back(Ku_traj_dat);
    }

    for(t = 0; t < Kv_data.size(); ++t)
    {
        //Kv_traj
        //Go through rows and columns of the message
        for(r = 0; r < Constants::num_controls_v; ++r)
        {
            for(c = 0; c < Constants::num_states; ++c)
            {
                Kv_traj_dat(r,c) = Kv_data[t].gain_mat[r].gain_list[c];
            }
        }

        this->drone_traj.Kv_traj.push_back(Kv_traj_dat);
    }

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

double ControlCalculator::clamp(double val, double min_val, double max_val)
{
    // Cap the value at maximum if it is too large
    if(val > max_val)
    {
        val = max_val;
    }
    // Make sure the value isn't too small
    else if (val < min_val)
    {
        val = min_val;
    }

    // Return a value that is in range now
    return val;
}
