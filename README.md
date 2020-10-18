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

CGAL depends on a library called [MPFR](https://www.mpfr.org/) which can be downloaded at [https://www.mpfr.org/mpfr-current/#download](https://www.mpfr.org/mpfr-current/#download). Once downloaded, extract the contents and navigate to the directory you extracted. Then, run the following commands:
```
./configure
make
make check
sudo make install
```

This will install MPFR in /usr/local and will be accessible by CGAL during compilation.

### Launch Instructions
Currently there are 3 different launch modes for this package: offline trajectory generation, drone simulation, and real-life flight. Each of these is detailed below.

Each launch file uses a standard set of arguments so the code doesn't need to be recompiled every time something small is changed. A description of these arguments is also below.

Note that while changing the arguments in the launch files removes the need to recompile, changing the intended trajectory requires a rebuild of the code. For example, if you want to change from a figure eight to an inclined circle, you should do this in the [gtddp_drone_target_trajectory](https://github.com/jkleiber/gtddp_drone_target_trajectory/tree/master) package and then rerun `catkin_make`

##### A. Offline Trajectory Generation
If you want to generate an offline trajectory, run the command that corresponds to the system you wish to use:
```
roslaunch gtddp_drone drone_traj_gen.launch
roslaunch gtddp_drone pursuit_traj_gen.launch
roslaunch gtddp_drone cart_pole_traj_gen.launch
```
This will generate 5 trajectory files in your home directory (x_traj.csv, u_traj.csv, v_traj.csv, Ku_traj.csv and Kv_traj.csv) that will be used by the flight controller when run in offline mode.

##### B. Simulation
If you want to run the simulator, run one of the following launch files:
```
roslaunch gtddp_drone drone_sim.launch
roslaunch gtddp_drone pursuit_sim.launch
roslaunch gtddp_drone cart_pole_sim.launch
```
This will bring up the simulator and run the code in either an offline or real-time state. Modify the `<arg>` tags in the launch file itself to choose which mode to operate in.

##### C. Real AR.Drone
This assumes you have followed the setup instructions for an ARDrone in the ARDrone_Control repository (or that you know what you're doing)
After determining the IP address of your AR.Drone and the IP and UDP port of the VICON Datastream SDK (and you've verified that you can connect to both at the same time) enter these into the `real_gtddp_drone.launch` launch file, located in the launch folder.
Make sure to set the rest of the `<arg>` tags appropriately based on your setup. Then run:
```
roslaunch gtddp_drone drone_real.launch
```
This has also been implemented for pursuit-evasion, but is still in its early days. The launch file for pursuit can be used by running:
```
roslaunch gtddp_drone pursuit_real.launch
```

### Launch File Arguments
* is_sim: boolean flag for if this is to be run in a simulation or not. 0 = real world (default), 1 = simulation.
* real_time: boolean flag for if the program expects pre-generated data (offline) or if it expects to optimize the trajectory in real time (online). 0 = offline, 1 = online (default).
* target_timestep: how much to step forward (in seconds) when pulling the next goal position. 0.5 seconds is the default.
* num_legs: the number of goal positions on a flight trajectory. 0 is the default.
* traj_gen: boolean flag for if this code will generate a trajectory or not. 0 = no (default), 1 = yes.

### Flight Instructions
The following is for drone simulation/real-life flights specifically. The cart-pole simulation runs automatically and has no real-life counterpart yet.

#### Keyboard Commands
G: "GO" (start the trajectory tracking)   
Spacebar: Land the drone   
Q: "Quit" (stop the keyboard node. Note: this does not stop the other nodes yet)   
A: Fly up   
Z: Fly down   
C: Fly forward (drone's perspective)   
X: Fly backward (drone's perspective)   
E: Fly left (drone's perspective)   
D: Fly right (drone's perspective)   
J: Turn left (drone's perspective)  
K: Turn right (drone's perspective)  
R: Reset drone (helpful for crashes)   
Ctrl+C: Shutdown software   
1: (Pursuit Only) Switch to drone 1 (pursuer) controls   
2: (Pursuit Only) Switch to drone 2 (evader) controls   

The drone will automatically take off and hover using its own internal system. Once it is in a stable hover state, press the G button on your keyboard (G is for GO) to start the drone on its path following adventure. If you ever want the drone to land, press spacebar.
To exit the program, press Q on the keyboard and then hit Ctrl-C like normal to kill the rest of the nodes. If you don't press Q first, the keyboard node might not be shutdown properly and you will have to close the terminal window to start a new flight.  
  
In pursuit mode, the spacebar will land both drones, while G and Q work as normal. All other commands are drone specific, so you should press either 1 or 2 to switch modes between drones. This acts as a toggle and does not need to be held down.

### Data Logging
All flight records are logged to CSV files in the home directory with a timestamp to avoid overwriting past logs. If the code crashes the logs may not be saved.

### Issue Tracking and Known Issues
This project is actively being developed and probably has bugs. These can be found under the issues tab. If you find any bugs or something doesn't work the way you think it should, please report these situations and identify the conditions necessary to replicate them.
