# gtddp_drone

### Overview
This ROS package implements Game-Theoretic Differential Dynamic Programming (GT-DDP) for an AR.Drone. With a little effort, this code can be adapted to run on any ROS enabled drone.

### Features
* GT-DDP trajectory optimizer
* AR.Drone controller
* Keyboard node for starts and landings

### Library Dependencies
* Boost - Used for numeric methods and solving
* CGAL - Used as a quadratic programming solver 
* Eigen - C++ matrix library

### ROS Package Dependencies
* ardrone_autonomy - Used in real-life flights, and a dependency of the simulator
* mocap_vicon - Used in real-life flights
* [gtddp_drone_target_trajectory](https://github.com/jkleiber/gtddp_drone_target_trajectory/tree/master) - Package that manages equations for the different flight trajectories. Uses an abstract class to make adding new trajectories easy. Contains some trajectories implemented using differential flatness. Links to gtddp_drone using a ROS service.
* [gtddp_drone_msgs](https://github.com/jkleiber/gtddp_drone_msgs/tree/master) - Contains the ROS msg and srv files used by gtddp_drone and gtddp_drone_target_trajectory

### Installation Instructions
This package must be installed alongside its dependencies. It is recommended to install this package as a submodule of the [ARDrone_Control](https://github.com/jkleiber/ARDrone_Control) repository because this is the only tested configuration at present.
However, you can install this package like any other ROS package by cloning it into your workspace:
```
git clone https://github.com/jkleiber/gtddp_drone.git
```
Make sure to install the above dependencies, otherwise gtddp_drone will not work.

#### CGAL Installation
```
sudo apt-get install libcgal-dev
sudo apt-get install libcgal-demo
```

### Launch Instructions
Currently there are 3 different launch modes for this package: offline trajectory generation, drone simulation, and real-life flight. Each of these is detailed below.

Each launch file uses a standard set of arguments so the code doesn't need to be recompiled every time something small is changed. A description of these arguments is also below.

Note that while changing the arguments in the launch files removes the need to recompile, changing the intended trajectory requires a rebuild of the code. For example, if you want to change from a figure eight to an inclined circle, you should do this in the [gtddp_drone_target_trajectory](https://github.com/jkleiber/gtddp_drone_target_trajectory/tree/master) package and then rerun `catkin_make`

##### A. Offline Trajectory Generation
If you want to generate a trajectory offline for the drone to follow, run the following command:
```
roslaunch gtddp_drone traj_gen.launch
```
This will generate 3 trajectory files in your home directory (x_traj.csv, u_traj.csv, and K_traj.csv) that will be used by the flight controller when run in offline mode.

##### B. Drone Simulation
If you want to run the drone in the simulator, run the following launch file:
```
roslaunch gtddp_drone gtddp_drone.launch
```
This will bring up the simulator and run the code in either an offline or real-time state. Modify the `<arg>` tags in the launch file itself to choose which mode to operate in.

##### C. Real AR.Drone
This assumes you have followed the setup instructions for an ARDrone in the ARDrone_Control repository (or that you know what you're doing)
After determining the IP address of your AR.Drone and the IP and UDP port of the VICON Datastream SDK (and you've verified that you can connect to both at the same time) enter these into the `real_gtddp_drone.launch` launch file, located in the launch folder.
Make sure to set the rest of the `<arg>` tags appropriately based on your setup. Then run:
```
roslaunch gtddp_drone real_gtddp_drone.launch
```

### Launch File Arguments
* is_sim: boolean flag for if this is to be run in a simulation or not. 0 = real world (default), 1 = simulation.
* real_time: boolean flag for if the program expects pre-generated data (offline) or if it expects to optimize the trajectory in real time (online). 0 = offline, 1 = online (default).
* target_timestep: how much to step forward (in seconds) when pulling the next goal position. 0.5 seconds is the default.
* num_legs: the number of goal positions on a flight trajectory. 0 is the default.
* traj_gen: boolean flag for if this code will generate a trajectory or not. 0 = no (default), 1 = yes.

### Flight Instructions
The drone will automatically take off and hover using its own internal system. Once it is in a stable hover state, press the G button on your keyboard (G is for GO) to start the drone on its path following adventure. If you ever want the drone to land, press spacebar.
To exit the program, press Q on the keyboard and then hit Ctrl-C like normal to kill the rest of the nodes. If you don't press Q first, the keyboard node might not be shutdown properly and you will have to close the terminal window to start a new flight.

### Data Logging
All flight records are logged to CSV files in the home directory with a timestamp to avoid overwriting past logs. If the code crashes the logs may not be saved.

### Issue Tracking and Known Issues
This project is actively being developed and has bugs. These can be found under the issues tab. If you find any bugs, please report them and identify the conditions necessary to replicate them.

