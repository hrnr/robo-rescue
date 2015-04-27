cmake_minimum_required(VERSION 2.8.3)
project(p3dx_urdf_model)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roslaunch
)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES p3dx_urdf_model
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

#############
## Install ##
#############

# install roslaunch files
install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

#############
## Testing ##
#############

# test all launch files
roslaunch_add_file_check(launch)
