# robo-rescue
ROS nodes for rescue robots

This project brings P3DX robot to [ROS](http://www.ros.org/). It contains
nodes for ROS which integrates with [VREP](http://www.coppeliarobotics.com/)
robotic simulator. But its modular design allows easy porting to other
simulators and even to real robots.

Although whole stack has been designed around VREP, it was designed with real
robots in mind, and should allow easy porting of algorithms to real robots.

# Building

This project contains standard ROS [catkin](http://wiki.ros.org/catkin)
packages, which is default build system in recent ROS releases. So to to build
you need to [install and configure](http://wiki.ros.org/ROS/Installation) your
ROS environment first.

This steps expects your ROS environment and VREP simulator are installed and configured properly.

Also if you want to use VREP bridge, you need [robo-rescue-simulation-
vrep](https://github.com/hrnr/robo-rescue-simulation-vrep) (this guide
includes steps to install it too).

1. clone this repository
```
	git clone http://github.com/hrnr/robo-rescue
```

3. link (or copy) VREP common bridge to your workspace (this is needed to
communicate with vrep)
```
	ln -s <VREP dir>/programming/ros_packages/vrep_common <catkin workspace>/src/vrep_common
```

4. add project packages to your workspace
```
	cp -r robo-rescue/src/* <catkin workspace>/src
```

5. build your catkin workspace
```
	cd <catkin workspace>
	catkin_make
```

You may also need to install other ROS packages such as `gmapping` and
`move_base`. This dependencies can be installed by
[rosdep](http://wiki.ros.org/rosdep).

# Running `p3dx_robot`

If you have retrieved and built all packages successfully you can run robot
with these steps.

2. clone repository containing VREP model and test enviroment
```
	git clone https://github.com/hrnr/robo-rescue-simulation-vrep
```

1. run `roscore` (this must be running before VREP)
```
	roscore
```

2. load scene and robot model in vrep
```
	File > open scene ... robo-rescue-simulation-vrep/test_scenes/level01.ttt
	File > load model ... robo-rescue-simulation-vrep/models/pioneer-p3dx.ttm
```

3. run simulation in VREP (PLAY	button)

4. run P3DX via roslaunch
```
	roslaunch p3dx_robot p3dx.launch
```

That's it, your P3DX robot is running in ROS

You can check that everything is running with `rostopic list` robot topics
starts with `/p3dx_0` since this is first PD3X in simulation. Check some topic
with `rostopic echo`, it should publish messages.

Run `rviz` for little demo. Load rviz config file from `src/pioneer-
p3dx/p3dx_robot/config/p3dx-config.rviz`. Set `2D Nav Goal` to robot, it will
go to specified location and map its environment. This is how robot looks in
its initial pose:

![p3dx_robot after installing](doc/rviz-initialpose.png?raw=true "p3dx_robot after installing in its initial pose")

# Architecture
![Hierarchy model](doc/Layers.png?raw=true)

This project uses layered architecture to create multiple layers of abstraction. 
Bottom layer is Hardware Abstraction Layer. With aim to hide maximum of hardware differences. Middle layer (DPL) aggregates multiple data streams from HAL into single data stream. 
Top layer holds higher algorithms which operate on the whole robot. Thanks to clearly defined layers is this architecture suitable for fast prototyping of robots.

## Hardware Abstraction Layer
![Hierarchy model](doc/HAL.png?raw=true)
### Interface robot <-> HAL
HAL receives raw data from sensors and sends raw low level commands to joints. Data may come from real robots as well as simulation. Nodes of HAL should transform data ,which come from variety of bus interfaces and sensor or actuator types. This layer is therefore uniquely build for each robot depending on its configuration. This layer also holds function of synchronizer. In case of simulation it should set ROS default time to simulated. Otherwise it should time-stamp data with ROS default system time.
### Interface HAL <-> DPL 
Messages send as an output from HAL must be time_stamped for use in future calculations. Every message should have set proper ID connecting submitted data with location of sensor on robots body (frame_id). Every time of sensor has predefined type of message which may publish.
 * Ultrasonic ,IR ,Bumpers : [sensor_msgs/Range](http://docs.ros.org/api/sensor_msgs/html/msg/Range.html)
 * Laser scanners :  [sensor_msgs/LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)
 * Joints ,Motors :  [sensor_msgs/JointState](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html)
 * Accelerometers, Gyroscopes, Magnetometers : [geometry_msgs/Vector3Stamped](http://docs.ros.org/api/geometry_msgs/html/msg/Vector3Stamped.html)

 Our HAL implements abstraction for PioneerP3dx robot simulated in V-rep.

 ## Data Processing Layer
 ![Hierarchy model](doc/DPL.png?raw=true)
 ### Overview
 DPL aggregates multiple data streams received from HAL. These messages must be fused together to provide overall robot state information. All data should be transformed to robot main frame_ID called base_link and then processed. Output from this stack of ROS packages should be come standardized ROS messages like Odom, PointCloud ,LaserScan. These messages are commonly used in algorithms. This layer also includes passing messages from algorithms to bottom layers. These are mostly represented by Twist commands for movement of the robot. Twist message is translated into angular velocities of robot motors.

 ## Algorithms
 This layer includes all algorithms needed for successful navigation of robot. It includes SLAM algorithm for map building and localization. It also needs to avoid obstacles and plan robot motion. Lastly This layer will include basic robot AI.

 ## TF Tree
This project use standard ROS TF tree. It is formation of coordinate frames of all vital parts of the robot. Robot model is described in URDF file.  TF uses forward and inverse transformation in between the frames. The main structure of the robot consists of frames: map -> odom -> base_footprint ->
base_link ->[robot structure ,sensors and actuators].

## Namespaces
Whole project use same hierarchical approach in naming. All names in [] are dynamically substituted.
Topics:
 * [robot_id]/topic_name : Algorithms namespace
 * [robot_id]/dpl/topic_name : DPL namespace
 * [robot_id]/dpl/hal/[node_name]/sensor[ID]/data type  :HAL namespace