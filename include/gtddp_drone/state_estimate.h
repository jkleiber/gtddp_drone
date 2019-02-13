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
#include <geometry_msgs/Vector3.h>

//User msgs
#include <gtddp_drone/state_data.h>

//Library msgs
#include <ardrone_autonomy/Navdata.h>
#include <vicon/Subject.h>

class StateEstimator
{
    public:
        StateEstimator();
        StateEstimator(ros::Publisher state_pub);

        //Subscriber callbacks
        void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_data)
        void navdata_callback(const ardrone_autonomy::Navdata::ConstPtr& navdata);
        void vicon_data_callback(const vicon::Subject::ConstPtr& vicon_subject);
        
        //Publisher callback
        void state_estimate_publish();

    private:
        //Messages
        gtddp_drone::state_data cur_state;

        //Publisher
        ros::Publisher state_pub;

        //Flags
        bool imu_init;
        bool navdata_init;
        bool vicon_init;
};

#endif