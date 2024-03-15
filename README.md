# QuadrupedalRobotswithB2
## Overview

This file document the control design of a quadrupdel robot with bounding gait with two flight phases.

Bounding, a gait prevalent among quadrupedal animals in nature, is characterized by pronounced pitching motion and significant hip movement. This is the process of energy accumulation and release by quadrupeds during high-speed movement.

instead of directly controlling the motion of the CoM of torso to move in a straight line at constant speed with zero body rotation, the robot was treated as two 
coupled parts and we focusing on the motion control of the stance leg while encouraging passive oscillation of the torso.
Three types of control schemes have been tested and compared thoroughly: conventional CoM driving, controller with LIP and model for hips, controller and SLIP
model for hips.

Contents:
* **Simualtion examples:** conventional CoM driving, controller with LIP and model for legs, controller and SLIP model for legs.
* **Hardware tests:** controller with LIP assumption for legs and controller with SLIP model for legs.

Features:

* **Hip motion:** instead of directly controlling the motion of the CoM of torso to move in a straight line at constant speed with zero body rotation, the robot was treated as two coupled parts and we focusing on the motion control of the stance leg while encouraging passive oscillation of the torso. 
* **Pitching motion:** promoting passive oscillation of the torso.

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Jing Cheng, Yasser G. Alqaham, Zhenyu Gan<br />
Affiliation: [DLAR Lab](https://dlarlab.syr.edu)<br />
Maintainer: Jing Cheng, jcheng13@syr.edu />**
With contributions by: Unitree(https://github.com/unitreerobotics/unitree_guide)

This projected was initially developed at Syracuse University (Dynamic Locomotion and Robotics Lab).

<img src="/Fig/Model_1.png" alt="Model" width="50%" height="50%">

## Publications

This work has been submitted to  IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2024).

If you use this work in an academic context, please cite the following publication:


## Requirements
### Environment
We recommand users to run this project in Ubuntu 18.04 and ROS melodic environment.
### Dependencies
2. [unitree_ros](https://github.com/unitreerobotics/unitree_ros)<br>
3. [unitree_legged_msgs](https://github.com/unitreerobotics/unitree_ros_to_real)(Note that: unitree_legged_real package should not be a part of dependencies)<br>This code requires a MATLAB version later than MATLAB R2019b.

## Usage

Readers can simply open 'Demo_Main.m' and hit the 'Run' button to follow the whole demo code.

Readers can also run any individual section if interested by hitting the 'Run Section' button, or navigate to the foler of each section and run 'Section_X_XXXX'.

## Summary

<img src="/Fig/Summary.gif" alt="Summary" width="100%" height="100%">

