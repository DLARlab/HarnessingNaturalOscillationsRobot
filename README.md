# HarnessingNaturalOscillationsRobot
## Video
https://github.com/DLARlab/HarnessingNaturalOscillationsRobot/assets/80359063/3f761554-0ca6-4148-bd3b-320e482e725c

## Overview

This file documents the control design of a quadrupedal called A1 from Unitree with a bounding gait with two flight phases.

Introduction:

Bounding, a gait prevalent among quadrupedal animals in nature, is characterized by pronounced pitching motion and significant hip movement. This is the process of energy accumulation and release by quadrupeds during high-speed movement.

Contents:
* **Simulation examples:** conventional CoM driving, controller with LIP and model for legs, controller and SLIP model for legs.
* **Hardware tests:** controller with LIP assumption for legs and controller with SLIP model for legs.

Features:

* **Hip motion:** Instead of directly controlling the motion of the torso’s CoM to move in a straight line at a constant speed with zero body rotation, the robot was treated as two coupled parts. We focused on the motion control of the stance leg while encouraging passive oscillation of the torso. 
* **Pitching motion:** promoting natural oscillations of the torso for High-Speed, Efficient Asymmetrical Locomotion.

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Jing Cheng, Yasser G. Alqaham, Zhenyu Gan<br />
Affiliation: [DLAR Lab](https://dlarlab.syr.edu)<br />
Maintainer: Jing Cheng, jcheng13@syr.edu />**
With contributions by: Unitree(https://github.com/unitreerobotics/unitree_guide)

This project was initially developed at Syracuse University (Dynamic Locomotion and Robotics Lab).

<img src="/Fig/fig1.svg" alt="Model" width="100%" height="100%">

## Publications

This work has been submitted to the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2024).

If you use this work in an academic context, please cite the following publication:

## Requirements
### Environment

We recommend that users run this project in Ubuntu 18.04 with ROS melodic or 20.04 with ROS noetic.

### Dependencies

Please place the three packages, unitree_guide, unitree_ros, and unitree_ros_to_real in our repository in a ROS workspace’s source directory.

## build

Open a terminal and switch the directory to the ros workspace containing unitree_guide, then run the following command to build the project:
```
catkin_make
```

## run

In the same terminal, run the following command step by step:
```
source ./devel/setup.bash
```
To open the gazebo simulator, run:
```
roslaunch unitree_guide gazeboSim.launch 
```

For starting the controller, open an another terminal and switch to the same directory,  then run the following command:
```
./devel/lib/unitree_guide/junior_ctrl
```

## Usage

### Simulation

After starting the controller,  the robot will lie on the ground of the simulator, then press the '2' key on the keyboard to switch the robot's finite state machine (FSM) from **Passive**(initial state) to **FixedStand**.  

Then press the '4' key to switch the FSM from **FixedStand** to **Bounding_CoM**, which corresponds to the first control scheme in the paper. Now, you can press the 'w' key to accelerate the robot. The desired velocity and actual velocity will show on the terminal.

Press the '2' to return to the **FixedStand**. The program automatically creates a plot including front/hip position, joint angle, foot position, ground reaction force, motor torque, etc. After you close all the figures, you will return to the **FixedStand**.

Go back to **FixedStand**, then press the ‘7’ key to switch the FSM from **FixedStand** to **Bounding_LIP**, which corresponds to the second control scheme in the paper. In this case, you don't need to press 'w' to accelerate the robot. The program will accelerate it by itself.

Go back to **FixedStand**, then press the ‘6’ key to switch the FSM from **FixedStand** to **Bounding_SLIP**, which corresponds to the third control scheme in the paper. In this case, you don't need to press 'w' to accelerate the robot. The program will accelerate the robot by itself.

(If there is no response, you need to click on the terminal opened to start the controller and then repeat the previous operation)

### Hardware implementation

Connect a cable to your computer with the robot or download the entire folder on your robot computer.

Assme you connect your robot with a cable, set the local IP address as 192.168.123.xxx. Then ping 192.168.123.161 to make sure your computer can communicate with robot.

In the CmakeLists.txt file, set REAL_ROBOT ON, set SIMULATION OFF, and set DEBUG OFF.

Open a terminal and switch the directory to the ros workspace containing unitree_guide,  then run the following command to build the project:
```
catkin_make
```
```
source ./devel/setup.bash
```
On the same terminal, run the following command with **root** right:
```
rosrun unitree_guide junior_ctrl
```

Use the remote controller to control the robot:

Press 'L2+B' to switch the robot's finite state machine to **Passive**.

Press 'L2+A' to switch the robot's finite state machine from **Passive** to **FixedStand**. 

Go back to **FixedStand**, then press the ‘L1+X key to switch the FSM from **FixedStand** to **Bounding_LIP**, which corresponds to the second control scheme in the paper.

Go back to **FixedStand**, then press the R2+B key to switch the FSM from **FixedStand** to **Bounding_SLIP**, which corresponds to the second control scheme in the paper. Be careful, the robot will accelerate very quickly.

The program will record the data automatically.

It is not recommended to test **Bounding_CoM** on the hardware at the moment because it has not been fully debugged.
