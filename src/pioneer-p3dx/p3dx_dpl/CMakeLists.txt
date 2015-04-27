cmake_minimum_required(VERSION 2.8.3)
project(p3dx_dpl)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  roslaunch
  tf
 )

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES p3dx_dpl
#  CATKIN_DEPENDS tf
#  DEPENDS system_lib
)

###########
## Build ##
###########
# c++11 support required
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(motor_controler src/motor_controler.cpp)
target_link_libraries(motor_controler ${catkin_LIBRARIES})

add_executable(imu_provider src/imu_provider.cpp)
target_link_libraries(imu_provider ${catkin_LIBRARIES})

add_executable(odometry_provider src/odometry_provider.cpp)
target_link_libraries(odometry_provider ${catkin_LIBRARIES})

add_executable(odom_tf_broadcaster src/odom_tf_broadcaster.cpp)
target_link_libraries(odom_tf_broadcaster ${catkin_LIBRARIES})

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
