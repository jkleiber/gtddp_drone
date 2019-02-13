#ifndef STATE_ESTIMATE_H
#define STATE_ESTIMATE_H

//System libs
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>

//User libs
#include "gtddp_drone/gtddp_lib/Constants.h"

//ROS msgs

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

//User msgs
#include <gtddp_drone/state_data.h>
#include <ardrone_autonomy/Navdata.h>
#include <vicon/Subject.h>

class StateEstimator
{
    public:
        StateEstimator();
        StateEstimator(ros::Publisher state_pub);

        void vicon_data_callback(const vicon::Subject::ConstPtr& vicon_subject);
        void navdata_callback(const ardrone_autonomy::Navdata::ConstPtr& navdata);
        void state_estimate_publish();

    private:
        //Messages
        gtddp_drone::state_data cur_state;

        //Publisher
        ros::Publisher state_pub;

        //Flags
        bool vicon_init;
};

#endif