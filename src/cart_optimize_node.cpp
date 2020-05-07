#include <ros/ros.h>

#include "gtddp_drone/gtddp_lib/Constants.h"
#include "gtddp_drone_msgs/Trajectory.h"
#include "gtddp_drone/traj_optimizer.h"

// TODO: finish this node so the new design pattern can be realized
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cart_optimize_node");

    ros::NodeHandle node;

    // Load Constants
    ConstantLoader loader(node);


}
