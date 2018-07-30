# baxter_kinematics_dynamics

This is a kinematics and dynamics library for Rethink Baxter, using orocos KDL. And this package is only tested on Ubuntu 14.04 STL + ROS indigo + Baxter SDK 1.2.0

## baxter_kdl
This package contains main functions of this library, including:

* forward kinematics
* inverse kinematics 
    * position only IK
    * 6d pose IK
* gravity effort
* inverse dynamics
* external torque estimation
* physical collision checking
    * scalable threshold

## baxter_kdl_test
An example package of using baxter_kdl

# Prerequisite

* orocos KDL

This library depends on orocos KDL (Kinematics and Dynamics Library). Thus, orocos-kdl package has to be installed.
```
sudo apt-get install ros-indigo-orocos-*
```

* Baxter SDK 1.2.0

This package is only tested on Baxter SDK 1.2.0, so you should set up a workstation with Baxter SDK 1.2.0, please refer to [Baxter Workstation Setup](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup) for more information.

# Install

```
1. cd ~/catkin_ws/src
2. git clone https://github.com/qqfly/baxter_kinematics_dynamics
3. cd ~/catkin_ws
4. source ~/[YourBaxterWorkspace]/devel/setup.bash
5. catkin_make
```

# How to use

The demo code can be found in baxter_kdl_test package. This library onle works for real robot.

<div  align="center">
    <img src="/baxter_kinematics_dynamics/pic/test_output.png" width = "600" height = "300" alt="output" />
</div>
<br>

# Author

Qiang Qiu

Shanghai Jiao Tong University

Wechat Official Account: Nao (ID: qRobotics)
