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

    //Set the optimizer to uninitialized
    this->initialized = false;

    //Set the mode to online optimization
    this->generation_mode = false;

    //Default to real time operation
    this->real_time = true;

    //Initialize logging
    this->logging_init();

    //Set the beginning time
    this->begin_time = ros::Time::now();

    //Reset number of legs
    this->num_legs = 0;
}


/**
 *
 */
Optimizer::Optimizer(ros::Publisher& traj_publisher,
                    ros::Publisher& state_publisher,
                    ros::Publisher& init_publisher,
                    ros::ServiceClient& target_client,
                    bool generate_traj,
                    bool real_time,
                    bool open_loop)
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

    //Set the optimizer to uninitialized
    this->initialized = false;

    //Select the optimization mode and real_time mode
    this->generation_mode = generate_traj;
    this->real_time = real_time;

    //Set up for trajectory generation if needed
    if(this->generation_mode)
    {
        //Set the current state to the origin
        this->cur_state.setZero();

        // If this is pursuit, add the second drone's start position
        if(!Constants::ddp_selector.compare("pursuit"))
        {
            this->cur_state(12) = 1;
        }

        // Send the initial conditions (all 0) to the target trajectory node
        // NOTE: In pursuit mode we don't care about the init pub, so this is fine the way it is
        for(int i = 0; i < Constants::num_states; ++i)
        {
            current_state.state.push_back(0.0);
        }

        //Initialize the target trajectory generator
        this->init_pub.publish(current_state);

        //Open all of the files
        this->open_genfiles(open_loop);
    }
    else if(!this->real_time)
    {
        std::cout << "Real-Time operation disabled! Using generated trajectory.\n";

        //Open all of the files
        this->open_genfiles(open_loop);
    }

    //Initialize logging if we are flying a drone
    if(!generation_mode)
    {
        this->logging_init();
    }

    //Set the beginning time
    this->begin_time = ros::Time::now();

    //Reset number of legs
    this->num_legs = 0;
}

/**
 * @brief Destroy the Optimizer:: Optimizer object
 *
 */
Optimizer::~Optimizer()
{
    //Close traj generation files
    this->x_traj_out.close();
    this->u_traj_out.close();
    this->v_traj_out.close();
    this->Ku_traj_out.close();
    this->Kv_traj_out.close();

    //Close the offline files
    this->x_traj_in.close();
    this->u_traj_in.close();
    this->v_traj_in.close();
    this->Ku_traj_in.close();
    this->Kv_traj_in.close();
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


void Optimizer::open_genfiles(bool open_loop)
{
    std::string x_file, u_file, v_file, Ku_file, Kv_file;
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
        printf("OFFLINE GENERATION FAILED TO FIND FILES!!!!!!!!\n");
        printf("OFFLINE GENERATION FAILED TO FIND FILES!!!!!!!!\n");
        printf("OFFLINE GENERATION FAILED TO FIND FILES!!!!!!!!\n");
        printf("OFFLINE GENERATION FAILED TO FIND FILES!!!!!!!!\n");
        printf("OFFLINE GENERATION FAILED TO FIND FILES!!!!!!!!\n");
        printf("OFFLINE GENERATION FAILED TO FIND FILES!!!!!!!!\n");
        return;
    }

    //Convert the home directory into a string
    std::string filepath(home_dir);

    //Add a / to the end of the home directory if needed
    if(home_dir[strlen(home_dir)-1] != '/')
    {
        filepath += "/";
    }

    //Find the file on the filepath
    x_file = filepath + "x_traj.csv";
    v_file = filepath + "v_traj.csv";
    Ku_file = filepath + "Ku_traj.csv";
    Kv_file = filepath + "Kv_traj.csv";

    // In open loop flights, we want a different file than normal
    if(open_loop)
    {
        u_file = filepath + "openloop_u_traj.csv";
    }
    else
    {
        u_file = filepath + "u_traj.csv";
    }

    //If trajectory generation is to occur, open the output files
    if(this->generation_mode)
    {
        this->x_traj_out.open(x_file);
        this->u_traj_out.open(u_file);
        this->v_traj_out.open(v_file);
        this->Ku_traj_out.open(Ku_file);
        this->Kv_traj_out.open(Kv_file);
    }
    //If an offline trajectory is expected, open the input files
    else if(!this->real_time)
    {
        this->x_traj_in.open(x_file);
        this->u_traj_in.open(u_file);
        this->v_traj_in.open(v_file);
        this->Ku_traj_in.open(Ku_file);
        this->Kv_traj_in.open(Kv_file);
    }

    std::cout << "Offline trajectory files opened!\n";
}


void Optimizer::set_num_legs(int legs)
{
    this->max_num_legs = legs;
}


/**
 *
 */
void Optimizer::traj_update_callback(const ros::TimerEvent& time_event)
{
    //Check to make sure current state and target state are initialized
    //If they are, then optimize the current trajectory
    if((this->cur_state_init && initialized) || (this->generation_mode && this->num_legs < this->max_num_legs))
    {
        //Create a service to update the current target
        gtddp_drone_msgs::target target_srv;

        //If the service succeeds, update the target and run the DDP
        if(target_client.call(target_srv))
        {
            printf("LEG #%d\n", num_legs);

            //Decode the target state from the service response
            this->target_state_decode(target_srv.response.target_state);

            //Update the DDP start and goals, then run the DDP loop to optimize the new trajectory
            ddpmain.update(this->cur_state, this->goal_state);

            //Optimize trajectory
            ddpmain.ddp_loop();

            //Update the last goal state to be the last state in the generated trajectory
            this->last_goal_state = ddpmain.get_x_traj().back();

            // Update the current state to be the last state
            this->cur_state = this->last_goal_state;

            //When not generating a trajectory offline (i.e. if you are in real time mode) then publish trajectory data
            if(!this->generation_mode)
            {
                //Publish the newly optimized trajectory data to the trajectory topic
                traj_pub.publish(this->get_traj_msg(ddpmain.get_x_traj(), ddpmain.get_u_traj(), ddpmain.get_v_traj(), ddpmain.get_Ku(), ddpmain.get_Kv()));
            }
            //Otherwise save the offline trajectory to a file
            else
            {
                this->write_traj_to_files(ddpmain.get_x_traj(), ddpmain.get_u_traj(), ddpmain.get_v_traj(), ddpmain.get_Ku(), ddpmain.get_Kv());
            }

            //Increment the leg counter
            this->num_legs++;
        }
        else if(this->generation_mode && this->num_legs >= this->max_num_legs)
        {
            printf("Trajectory generation complete!!\n");
        }
        //Otherwise notify user upon failure
        else
        {
            printf("ERROR!!! Target service failed!!!\n");
        }

    }
}


void Optimizer::pursuit_traj_callback(const ros::TimerEvent& time_event)
{
    if(this->generation_mode && this->num_legs < this->max_num_legs)
    {
        printf("LEG #%d\n", num_legs);

        //Update the DDP start and goals, then run the DDP loop to optimize the new trajectory
        ddpmain.update(this->cur_state, this->goal_state);

        //Optimize trajectory
        ddpmain.ddp_loop();

        //Update the last goal state to be the last state in the generated trajectory
        this->last_goal_state = ddpmain.get_x_traj().back();

        // Update the current state to be the last state
        this->cur_state = this->last_goal_state;

        // Save offline trajectory
        this->write_traj_to_files(ddpmain.get_x_traj(), ddpmain.get_u_traj(), ddpmain.get_v_traj(), ddpmain.get_Ku(), ddpmain.get_Kv());

        //Increment the leg counter
        this->num_legs++;
    }
    // End the trajectory generation program once the trajectory is generated
    else if(this->generation_mode)
    {
        exit(0);
    }
}


void Optimizer::offline_traj_callback(const ros::TimerEvent& time_event)
{
    //Declare local variables
    std::vector<Eigen::VectorXd> x_traj;
    std::vector<Eigen::VectorXd> u_traj;
    std::vector<Eigen::VectorXd> v_traj;
    std::vector<Eigen::MatrixXd> Ku_traj;
    std::vector<Eigen::MatrixXd> Kv_traj;
    Eigen::VectorXd temp_vector;
    Eigen::MatrixXd temp_matrix;
    std::string cell;
    std::string line;
    int cell_idx;
    int tmp_row;
    int tmp_col;

    //Only publish if the drone is ready to fly
    if(initialized)
    {
        std::cout << "Reading Leg #" << num_legs << std::endl;
        ++num_legs;

        //init temp vector for a state vector
        temp_vector.resize(Constants::num_states);
        temp_vector.setZero();

        //Read the x_traj file
        for(int i = 0; i < Constants::offline_traj_batch_size; ++i)
        {
            //Check to see if this file is OK to read
            if(!x_traj_in.good())
            {
                break;
            }

            //Get the next line from the CSV file
            getline(x_traj_in, line);
            //printf("1\n");

            //Process the file using comma as a delimiter
            std::stringstream ss(line);
            //printf("2\n");

            //Reset the cell counter
            cell_idx = 0;

            //Read each column in the line
            while(getline(ss, cell, ','))
            {
                //printf("3\n");
                temp_vector(cell_idx) = std::atof(cell.c_str());
                //printf("4\n");

                //Apply the x, y, z offsets
                //X
                if(cell_idx == 0)
                {
                    temp_vector(cell_idx) += x_offset;
                }
                //Y
                else if(cell_idx == 1)
                {
                    temp_vector(cell_idx) += y_offset;
                }
                //Z
                else if(cell_idx == 2)
                {
                    temp_vector(cell_idx) += z_offset;
                }
                //printf("5\n");

                //Increment the cell index
                cell_idx++;
            }

            //Push the temporary Eigen vector to the x traj c++ vector
            x_traj.push_back(temp_vector);
        }

        //std::cout <<"x_traj ready!\n";

        //init the temp vector for a control vector
        temp_vector.resize(Constants::num_controls_u);
        temp_vector.setZero();

        //Read the u_traj file
        for(int i = 0; i < Constants::offline_traj_batch_size; ++i)
        {
            //Check to see if this file is OK to read
            if(!u_traj_in.good())
            {
                break;
            }

            //Get the next line from the CSV file
            getline(u_traj_in, line);

            //Process the file using comma as a delimiter
            std::stringstream ss(line);

            //Reset the cell counter
            cell_idx = 0;

            //Read each column in the line
            while(getline(ss, cell, ','))
            {
                temp_vector(cell_idx) = std::stod(cell, 0);
                cell_idx++;
            }

            //Push the temporary Eigen vector to the u traj c++ vector
            u_traj.push_back(temp_vector);
        }

        //std::cout <<"u_traj ready!\n";

        //init the temp vector for a control vector
        temp_vector.resize(Constants::num_controls_v);
        temp_vector.setZero();

        //Read the u_traj file
        for(int i = 0; i < Constants::offline_traj_batch_size; ++i)
        {
            //Check to see if this file is OK to read
            if(!v_traj_in.good())
            {
                break;
            }

            //Get the next line from the CSV file
            getline(v_traj_in, line);

            //Process the file using comma as a delimiter
            std::stringstream ss(line);

            //Reset the cell counter
            cell_idx = 0;

            //Read each column in the line
            while(getline(ss, cell, ','))
            {
                temp_vector(cell_idx) = std::stod(cell, 0);
                cell_idx++;
            }

            //Push the temporary Eigen vector to the u traj c++ vector
            v_traj.push_back(temp_vector);
        }

        //Init the temp matrix for a gain matrix
        temp_matrix.resize(Constants::num_controls_u, Constants::num_states);
        temp_matrix.setZero();

        //Read the Ku_traj file
        for(int i = 0; i < Constants::offline_traj_batch_size; ++i)
        {
            //Check to see if this file is OK to read
            if(!Ku_traj_in.good())
            {
                break;
            }

            //Get the next line from the CSV file
            getline(Ku_traj_in, line);

            //Process the file using comma as a delimiter
            std::stringstream ss(line);

            //Reset the cell counter
            tmp_row = 0;
            tmp_col = 0;

            //Read each column in the line
            while(getline(ss, cell, ','))
            {
                temp_matrix(tmp_row, tmp_col) = std::stod(cell, 0);

                //Increment the row or column appropriately
                if(tmp_col == (Constants::num_states - 1))
                {
                    tmp_col = 0;
                    tmp_row++;
                }
                else
                {
                    tmp_col++;
                }
            }

            //Push the temporary Eigen matrix to the K traj c++ vector
            Ku_traj.push_back(temp_matrix);
        }

        //std::cout <<"K_traj ready!\n";

        //Init the temp matrix for a gain matrix
        temp_matrix.resize(Constants::num_controls_u, Constants::num_states);
        temp_matrix.setZero();

        //Read the Ku_traj file
        for(int i = 0; i < Constants::offline_traj_batch_size; ++i)
        {
            //Check to see if this file is OK to read
            if(!Kv_traj_in.good())
            {
                break;
            }

            //Get the next line from the CSV file
            getline(Kv_traj_in, line);

            //Process the file using comma as a delimiter
            std::stringstream ss(line);

            //Reset the cell counter
            tmp_row = 0;
            tmp_col = 0;

            //Read each column in the line
            while(getline(ss, cell, ','))
            {
                temp_matrix(tmp_row, tmp_col) = std::stod(cell, 0);

                //Increment the row or column appropriately
                if(tmp_col == (Constants::num_states - 1))
                {
                    tmp_col = 0;
                    tmp_row++;
                }
                else
                {
                    tmp_col++;
                }
            }

            //Push the temporary Eigen matrix to the K traj c++ vector
            Kv_traj.push_back(temp_matrix);
        }

        //Publish this data
        if(x_traj.size() > 0)
        {
            this->traj_pub.publish(this->get_traj_msg(x_traj, u_traj, v_traj, Ku_traj, Kv_traj));
        }
    }
}

void Optimizer::open_loop_traj_callback(const ros::TimerEvent& time_event)
{
    std::vector<Eigen::VectorXd> u_traj;
    Eigen::VectorXd temp_vector;
    std::string cell;
    std::string line;
    int cell_idx;
    gtddp_drone_msgs::Trajectory traj_msg;   //trajectory message result

    // Only publish if the drone is ready to fly
    if(initialized)
    {
        std::cout << "Reading Open Loop Leg #" << num_legs << std::endl;
        ++num_legs;

        // init the temp vector for a control vector
        temp_vector.resize(Constants::num_controls_u);
        temp_vector.setZero();

        //Read the u_traj file
        for(int i = 0; i < Constants::offline_traj_batch_size; ++i)
        {
            //Check to see if this file is OK to read
            if(!u_traj_in.good())
            {
                break;
            }

            //Get the next line from the CSV file
            getline(u_traj_in, line);

            //Process the file using comma as a delimiter
            std::stringstream ss(line);

            //Reset the cell counter
            cell_idx = 0;

            //Read each column in the line
            while(getline(ss, cell, ','))
            {
                temp_vector(cell_idx) = std::stod(cell, 0);
                cell_idx++;
            }

            //Push the temporary Eigen vector to the u traj c++ vector
            u_traj.push_back(temp_vector);
        }

        //Loop through each time step and encode the data into ROS messages
        //Note: the ddp only initializes values from 0 to num_time_steps - 1. Thus, 100 timesteps will yield 99 values
        for(int i = 0; i < u_traj.size(); ++i)
        {
            traj_msg.u_traj.push_back(this->get_ctrl_data_msg(u_traj, i));
        }

        // Publish open loop control signal
        if(u_traj.size() > 0)
        {
            this->traj_pub.publish(traj_msg);
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
        this->current_state.state[i] = this->cur_state(i);
    }

    //Publish the debugging current state info
    this->state_pub.publish(this->current_state);
}



/**
 *
 */
void Optimizer::target_state_decode(const gtddp_drone_msgs::state_data& target_event)
{
    printf("TARGET: [");
    for(int i = 0; i < Constants::num_states; ++i)
    {
        this->goal_state(i) = target_event.state[i];

        printf("%f ", this->goal_state(i));
    }
    printf("]\n");

    this->last_goal_state_init = true;
}


void Optimizer::init_optimizer(const std_msgs::Empty::ConstPtr& init_msg)
{
    //IF the optimizer has not been initialized yet, edit the target trajectory settings
    if(!initialized && this->real_time)
    {
        //Initialize the target trajectory generator
        this->init_pub.publish(current_state);

        double cur_time = (ros::Time::now() - begin_time).toSec();

        //Log the initial conditions
        std::cout << "REAL TIME OPERATION ENABLED\n";
        std::string data_str = std::to_string(cur_time) + "," + std::to_string(cur_state(0)) + "," + std::to_string(cur_state(1)) + "," + std::to_string(cur_state(2)) + "\n";
        this->init_data << data_str;

        //Set the optimizer as initialized
        this->initialized = true;
        printf("Optimization started!\n");
    }
    //If the optimizer hasn't been initialized, set the offset from the offline trajectory
    else if(!initialized && !this->real_time)
    {
        //Set the offsets
        x_offset = cur_state(0);
        y_offset = cur_state(1);
        z_offset = cur_state(2);

        double cur_time = (ros::Time::now() - begin_time).toSec();

        //Log the initial conditions
        std::string data_str = std::to_string(cur_time) + "," + std::to_string(cur_state(0)) + "," + std::to_string(cur_state(1)) + "," + std::to_string(cur_state(2)) + "\n";
        std::cout << "INITIAL CONDITIONS: " << data_str << std::endl;
        this->init_data << data_str;

        //Set the optimizer as initialized
        this->initialized = true;
        printf("Optimization started!\n");
    }

    //Close init logging
    this->init_data.close();
}



/**
 *
 */
gtddp_drone_msgs::Trajectory Optimizer::get_traj_msg(std::vector<Eigen::VectorXd> x_traj,
                                                     std::vector<Eigen::VectorXd> u_traj,
                                                     std::vector<Eigen::VectorXd> v_traj,
                                                     std::vector<Eigen::MatrixXd> Ku_traj,
                                                     std::vector<Eigen::MatrixXd> Kv_traj)
{
    //Declare local variables
    int i;                              //iteration variable
    gtddp_drone_msgs::Trajectory traj_msg;   //trajectory message result

    //Loop through each time step and encode the data into ROS messages
    //Note: the ddp only initializes values from 0 to num_time_steps - 1. Thus, 100 timesteps will yield 99 values
    for(i = 0; i < x_traj.size(); ++i)
    {
        traj_msg.x_traj.push_back(this->get_state_data_msg(x_traj, i));
        traj_msg.u_traj.push_back(this->get_ctrl_data_msg(u_traj, i));
        traj_msg.v_traj.push_back(this->get_ctrl_data_msg(v_traj, i));
        traj_msg.Ku_traj.push_back(this->get_gain_data_msg(Ku_traj, i));
        traj_msg.Kv_traj.push_back(this->get_gain_data_msg(Kv_traj, i));
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

    state_msg.state.resize(Constants::num_states);
    //Loop through each state variable for this particular state to extract the value
    for(int i = 0; i < Constants::num_states; ++i)
    {
        state_msg.state[i] = (x_traj[idx](i));
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



void Optimizer::write_traj_to_files(std::vector<Eigen::VectorXd> x_traj,
                                    std::vector<Eigen::VectorXd> u_traj,
                                    std::vector<Eigen::VectorXd> v_traj,
                                    std::vector<Eigen::MatrixXd> Ku_traj,
                                    std::vector<Eigen::MatrixXd> Kv_traj)
{
    //CSV output string
    std::string output;

    //Append the most recent x_traj to the x trajectory file
    for(int i = 0; i < x_traj.size(); ++i)
    {
        //Reset output
        output = "";

        //Go state by state
        for(int s = 0; s < Constants::num_states; ++s)
        {
            //Add the data
            output += std::to_string(x_traj[i](s));

            //Add the comma after all but the last element
            if(s != (Constants::num_states - 1))
            {
                output += ",";
            }
            else
            {
                output += "\n";
            }
        }

        //Append trajectory
        x_traj_out << output;
    }

    //Append the most recent u_traj to the u trajectory file
    for(int i = 0; i < u_traj.size(); ++i)
    {
        //Reset output
        output = "";

        //Go state by state
        for(int s = 0; s < Constants::num_controls_u; ++s)
        {
            //Add the data
            output += std::to_string(u_traj[i](s));

            //Add the comma after all but the last element
            if(s != (Constants::num_controls_u - 1))
            {
                output += ",";
            }
            else
            {
                output += "\n";
            }
        }

        //Append trajectory
        u_traj_out << output;
    }

    //Append the most recent v_traj to the v trajectory file
    for(int i = 0; i < v_traj.size(); ++i)
    {
        //Reset output
        output = "";

        //Go state by state
        for(int s = 0; s < Constants::num_controls_u; ++s)
        {
            //Add the data
            output += std::to_string(v_traj[i](s));

            //Add the comma after all but the last element
            if(s != (Constants::num_controls_v - 1))
            {
                output += ",";
            }
            else
            {
                output += "\n";
            }
        }

        //Append trajectory
        v_traj_out << output;
    }

    //Append the most recent Ku traj to the Ku trajectory file
    for(int i = 0; i < u_traj.size(); ++i)
    {
        //Reset output
        output = "";

        //Add data in row major order
        for(int r = 0; r < Constants::num_controls_u; ++r)
        {
            for(int c = 0; c < Constants::num_states; ++c)
            {
                //Add the data
                output += std::to_string(Ku_traj[i](r,c));

                //Add the comma after all but the last element
                if(!(r == (Constants::num_controls_u - 1) && c == (Constants::num_states - 1)))
                {
                    output += ",";
                }
                else
                {
                    output += "\n";
                }
            }
        }

        //Append trajectory
        Ku_traj_out << output;
    }

    //Append the most recent K traj to the K trajectory file
    for(int i = 0; i < u_traj.size(); ++i)
    {
        //Reset output
        output = "";

        //Add data in row major order
        for(int r = 0; r < Constants::num_controls_u; ++r)
        {
            for(int c = 0; c < Constants::num_states; ++c)
            {
                //Add the data
                output += std::to_string(Kv_traj[i](r,c));

                //Add the comma after all but the last element
                if(!(r == (Constants::num_controls_u - 1) && c == (Constants::num_states - 1)))
                {
                    output += ",";
                }
                else
                {
                    output += "\n";
                }
            }
        }

        //Append trajectory
        Kv_traj_out << output;
    }

    std::cout << "Trajectory files written\n";
}
