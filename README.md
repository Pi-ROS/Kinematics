# Kinematics

C++ implementation of the kinematics for an UR5.

## Getting started

To get started, clone this repository as a catkin package and in your workspace root, issue
```
$ catkin_make install
```

After that, launch [ur5_generic.py](https://github.com/mfocchi/robot_control/blob/10bc564604a3337b2cc38cc8555d0cadefccc7e4/lab_exercises/lab_palopoli/ur5_generic.py) file (you'll need the [Locosim framework](https://github.com/mfocchi/locosim)) and in another terminal shell, run
```
$ rosrun kinematics kinematics_node
```

## Module organization

The source code is in the src[src] directory.
* The [kinematics.cpp](src/kinematics.cpp) file contains the main components of the program.
* The [robot.cpp](src/robot.cpp) file contains the implementation of the robot class.
* The [controller.cpp](src/robot.cpp) file contains the implementation of the controller algorithms.
* The [ros.cpp](src/ros.cpp) file and [utils.cpp](src/utils.cpp) file contain the implementation of some utilities (respectively ros management and math utils functions).
* The [tasks](src/tasks) directory contains the code for each tasks, which are called by the main function in [kinematics.cpp](src/kinematics.cpp).

The [include](include) directory contains all the headers files (.hpp).

## Doxygen documentation
To generate the HTML version of the documentation, you'll need the [Doxygen tool](https://www.doxygen.nl/index.html).
After installing the tool, issue
```
$ doxygen
```

## Running tasks 3 and 4

In tasks 3 and 4, we employed a virtual link attacher in order to simulate the correct behavior of the gripper. Dowload into your CatKin workspace the Gazebo service code at [this link](https://github.com/PiJoint/gazebo_link_attacher), and add the following line to your `.world` file.

```
<plugin name="ros_link_attacher_plugin" filename="libgazebo_ros_link_attacher.so"/>
```
