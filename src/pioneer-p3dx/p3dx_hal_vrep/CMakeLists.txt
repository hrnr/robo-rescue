cmake_minimum_required(VERSION 2.8.3)
project(p3dx_hal_vrep)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  roslaunch
  vrep_common
)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES p3dx_hal_vrep
  CATKIN_DEPENDS roscpp vrep_common
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

add_executable(laser src/laser.cpp)
target_link_libraries(laser ${catkin_LIBRARIES})

add_executable(rightMotor src/rightMotor.cpp)
target_link_libraries(rightMotor ${catkin_LIBRARIES})

add_executable(leftMotor src/leftMotor.cpp)
target_link_libraries(leftMotor ${catkin_LIBRARIES})

add_executable(accelerometer src/accelerometer.cpp)
target_link_libraries(accelerometer ${catkin_LIBRARIES})

add_executable(gyroscope src/gyroscope.cpp)
target_link_libraries(gyroscope ${catkin_LIBRARIES})

add_executable(ultrasonic src/ultrasonic.cpp)
target_link_libraries(ultrasonic ${catkin_LIBRARIES})

add_executable(bumper src/bumper.cpp)
target_link_libraries(bumper ${catkin_LIBRARIES})

add_executable(clockServer src/clockServer.cpp)
add_dependencies(clockServer vrep_common_generate_messages_cpp)
target_link_libraries(clockServer ${catkin_LIBRARIES})

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
