cmake_minimum_required(VERSION 2.8.3)
project(gtddp_drone)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  mocap_vicon
  gtddp_drone_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(CGAL REQUIRED COMPONENTS Core)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   ctrl_data.msg
#   gain_vector.msg
#   gain_data.msg
#   state_data.msg
#   Trajectory.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   nav_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
# LIBRARIES gtddp_drone
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime mocap_vicon gtddp_drone_msgs
  DEPENDS cgal
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
)

include(${CGAL_USE_FILE})

## Declare a C++ library
add_library(GTDDP_LIB
  src/gtddp_lib/Constants.cpp
  src/gtddp_lib/DDP_main_mm.cpp
  src/gtddp_lib/DDP_Optimizer.cpp
  src/gtddp_lib/cost_functions/SingleQuadrotorCost.cpp
  src/gtddp_lib/cost_functions/PursuitCost.cpp
  src/gtddp_lib/optimizers/CC_DDP_optimizer.cpp
  src/gtddp_lib/optimizers/CC_DDP_optimizer_new.cpp
  src/gtddp_lib/optimizers/GT_DDP_optimizer.cpp
  src/gtddp_lib/optimizers/Pursuit_optimizer.cpp
  src/gtddp_lib/systems/PursuitDrones.cpp
  src/gtddp_lib/systems/Quadrotor.cpp
  src/gtddp_lib/systems/CartPole.cpp
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(gtddp_control_node src/gtddp_control_node.cpp src/control_calc.cpp src/flight_controller.cpp)
add_executable(pursuit_control_node src/pursuit_control_node.cpp src/control_calc.cpp src/flight_controller.cpp)
add_executable(cart_control_node src/cart_control_node.cpp src/control_calc.cpp src/flight_controller.cpp)
add_executable(gtddp_optimize_node src/gtddp_optimize_node.cpp src/traj_optimizer.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(gtddp_control_node ${catkin_LIBRARIES} GTDDP_LIB ${CGAL_LIBRARIES})
target_link_libraries(pursuit_control_node ${catkin_LIBRARIES} GTDDP_LIB ${CGAL_LIBRARIES})
target_link_libraries(cart_control_node ${catkin_LIBRARIES} GTDDP_LIB ${CGAL_LIBRARIES})
target_link_libraries(gtddp_optimize_node ${catkin_LIBRARIES} GTDDP_LIB ${CGAL_LIBRARIES})






#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gtddp_drone.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
