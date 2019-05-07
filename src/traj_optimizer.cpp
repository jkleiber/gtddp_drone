#include "gtddp_drone/traj_optimizer.h"


/**
 * 
 */
Optimizer::Optimizer()
{
    //Flag the state data as uninitialized
    this->cur_state_init = false;
    this->last_goal_state_init = false;

    //Initialize current state size
    this->cur_state.resize(Constants::num_states);
    this->goal_state.resize(Constants::num_states);
    this->last_goal_state.resize(Constants::num_states);

    //Set the control status to unknown
    this->ctrl_status = -1;

    //Set the optimizer to uninitialized
    this->initialized = false;

    //Initialize logging
    this->logging_init();

    //Set the beginning time
    this->begin_time = ros::Time::now();
}


/**
 * 
 */
Optimizer::Optimizer(ros::Publisher& traj_publisher, ros::Publisher& state_publisher, ros::Publisher& init_publisher, ros::ServiceClient& target_client)
{
    this->traj_pub = traj_publisher;
    this->state_pub = state_publisher;
    this->init_pub = init_publisher;
    this->target_client = target_client;
    
    //Flag the state data as uninitialized
    this->cur_state_init = false;
    this->last_goal_state_init = false;

    //Initialize current state size
    this->cur_state.resize(Constants::num_states);
    this->goal_state.resize(Constants::num_states);
    this->last_goal_state.resize(Constants::num_states);

    //Set the control status to unknown
    this->ctrl_status = -1;

    //Set the optimizer to uninitialized
    this->initialized = false;

    //Initialize logging
    this->logging_init();

    //Set the beginning time
    this->begin_time = ros::Time::now();
}

/**
 * @brief Destroy the Optimizer:: Optimizer object
 * 
 */
Optimizer::~Optimizer()
{
    this->init_data.close();
}


/**
 * @brief 
 * 
 */
void Optimizer::logging_init()
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
        printf("OPTIMIZER LOGGING FAILED TO REGISTER!!!!!!!!\n");
        printf("OPTIMIZER LOGGING FAILED TO REGISTER!!!!!!!!\n");
        printf("OPTIMIZER LOGGING FAILED TO REGISTER!!!!!!!!\n");
        printf("OPTIMIZER LOGGING FAILED TO REGISTER!!!!!!!!\n");
        printf("OPTIMIZER LOGGING FAILED TO REGISTER!!!!!!!!\n");
        printf("OPTIMIZER LOGGING FAILED TO REGISTER!!!!!!!!\n");
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
    ss << filepath << timestamp << "_init_data" << ".csv";
    ss >> filename;
    std::cout << "Logging initial conditions to " << filename << std::endl; 

    //Open the ground truth log file
    this->init_data.open(filename);
}


/**
 * 
 */
void Optimizer::traj_update_callback(const ros::TimerEvent& time_event)
{
    //Check to make sure current state and target state are initialized
    //If they are, then optimize the current trajectory
    if(this->cur_state_init && initialized && this->ctrl_status == gtddp_drone_msgs::Status::IDLE)
    {
        //Create a service to update the current target
        gtddp_drone_msgs::target target_srv;
        
        //If the service succeeds, update the target and run the DDP
        if(target_client.call(target_srv))
        {
            //Decode the target state from the service response
            this->target_state_decode(target_srv.response.target_state);

            //Update the DDP start and goals, then run the DDP loop to optimize the new trajectory
            ddpmain.update(this->cur_state, this->goal_state);

            //Update the last goal state to be the current goal state
            //this->last_goal_state = goal_state;

            //Optimize trajectory
            ddpmain.ddp_loop();

            //Publish the newly optimized trajectory data to the trajectory topic
            traj_pub.publish(this->get_traj_msg(ddpmain.get_x_traj(), ddpmain.get_u_traj(), ddpmain.get_Ku()));

            //Set ctrl status to unknown so the controller can start flying
            this->ctrl_status = -1;
        }
        //Otherwise notify user upon failure
        else
        {
            printf("ERROR!!! Target service failed!!!\n");
        }
        
    }
}


/**
 * 
 */
void Optimizer::state_estimate_callback(const nav_msgs::Odometry::ConstPtr& odom)
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

    //Linear velocity
    this->cur_state(3) = odom->twist.twist.linear.x;
    this->cur_state(4) = odom->twist.twist.linear.y;
    this->cur_state(5) = odom->twist.twist.linear.z;

    //Angular velocity
    this->cur_state(9) = odom->twist.twist.angular.x;
    this->cur_state(10) = odom->twist.twist.angular.y;
    this->cur_state(11) = odom->twist.twist.angular.z;

    //Set the last goal to the current state if this is the first leg
    if(!last_goal_state_init)
    {
        this->last_goal_state = this->cur_state;
    }

    //Set the current state as initialized
    this->cur_state_init = true;

    //Decode the current state
    for(int i = 0; i < Constants::num_states; ++i)
    {
        this->current_state.states[i] = this->cur_state(i);
    }

    //Publish the debugging current state info
    this->state_pub.publish(this->current_state);
}

/**
 * 
 */
/*
void Optimizer::state_estimate_callback(const tum_ardrone::filter_state::ConstPtr& estimate_event)
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

    //Set the last goal to the current state if this is the first leg
    if(!last_goal_state_init)
    {
        this->last_goal_state = this->cur_state;
    }

    //Set the current state as initialized
    this->cur_state_init = true;
}*/


/**
 * 
 */
void Optimizer::status_callback(const gtddp_drone_msgs::Status::ConstPtr& status)
{
    this->ctrl_status = status->status;
}


/**
 * 
 */
void Optimizer::target_state_decode(const gtddp_drone_msgs::state_data& target_event)
{
    printf("TARGET: [");
    for(int i = 0; i < Constants::num_states; ++i)
    {
        this->goal_state(i) = target_event.states[i];

        printf("%f ", this->goal_state(i));
    }
    printf("]\n");

    this->last_goal_state_init = true;
}


void Optimizer::init_optimizer(const std_msgs::Empty::ConstPtr& init_msg)
{
    //IF the optimizer has not been initialized yet, edit the target trajectory settings
    if(!initialized)
    {
        //Initialize the target trajectory generator
        this->init_pub.publish(current_state);

        double cur_time = (ros::Time::now() - begin_time).toSec();

        //Log the initial conditions
        std::string data_str = std::to_string(cur_time) + "," + std::to_string(cur_state(0)) + "," + std::to_string(cur_state(1)) + "," + std::to_string(cur_state(2)) + "\n";
        this->init_data << data_str;

        //Set the optimizer as initialized
        this->initialized = true;
        printf("Optimization started!\n");
    }
}



/**
 * 
 */
gtddp_drone_msgs::Trajectory Optimizer::get_traj_msg(std::vector<Eigen::VectorXd> x_traj, std::vector<Eigen::VectorXd> u_traj, std::vector<Eigen::MatrixXd> K_traj)
{
    //Declare local variables
    int i;                              //iteration variable
    gtddp_drone_msgs::Trajectory traj_msg;   //trajectory message result

    //Loop through each time step and encode the data into ROS messages
    //Note: the ddp only initializes values from 0 to num_time_steps - 1. Thus, 100 timesteps will yield 99 values
    for(i = 0; i < Constants::num_time_steps - 1; ++i)
    {
        traj_msg.x_traj.push_back(this->get_state_data_msg(x_traj, i));
        traj_msg.u_traj.push_back(this->get_ctrl_data_msg(u_traj, i));
        traj_msg.K_traj.push_back(this->get_gain_data_msg(K_traj, i));
    }

    //Set up the Header of this message (for time tracking)
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.header.frame_id = "/world";

    //Return the generated message
    return traj_msg;
}


/**
 * 
 */
gtddp_drone_msgs::state_data Optimizer::get_state_data_msg(std::vector<Eigen::VectorXd> x_traj, int idx)
{
    //Set up a state data message
    gtddp_drone_msgs::state_data state_msg;

    //Loop through each state variable for this particular state to extract the value
    for(int i = 0; i < Constants::num_states; ++i)
    {
        state_msg.states[i] = (x_traj[idx](i));
    }

    //Return the generated message
    return state_msg;
}


/**
 * 
 */
gtddp_drone_msgs::ctrl_data Optimizer::get_ctrl_data_msg(std::vector<Eigen::VectorXd> u_traj, int idx)
{
    //Set up a state data message
    gtddp_drone_msgs::ctrl_data ctrl_msg;

    //Loop through each state variable for this particular state to extract the value
    for(int i = 0; i < Constants::num_controls_u; ++i)
    {
        ctrl_msg.ctrl[i] = u_traj[idx](i);
    }

    //Return the generated message
    return ctrl_msg;
}


/**
 * 
 */
gtddp_drone_msgs::gain_data Optimizer::get_gain_data_msg(std::vector<Eigen::MatrixXd> K_traj, int idx)
{
    //Set up the gain data message types
    gtddp_drone_msgs::gain_data gain_msg;
    gtddp_drone_msgs::gain_vector gain_row;

    //Declare local variables
    int r, c;

    //Loop through the gain matrix for this particular state
    for(r = 0; r < Constants::num_controls_u; ++r)
    {
        //Clear old row  data
        gain_row.gain_list.clear();

        //Add all the values to the row vector
        for(c = 0; c < Constants::num_states; ++c)
        {
            gain_row.gain_list.push_back(K_traj[idx](r,c));
        }

        //Save the row
        gain_msg.gain_mat.push_back(gain_row);
    }

    //Return the processed gain message
    return gain_msg;
}
