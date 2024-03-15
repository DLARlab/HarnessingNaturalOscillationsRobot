/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>

enum class CtrlPlatform{
    GAZEBO,
    REALROBOT,
};

enum class RobotType{
    A1,
    Go1
};

enum class UserCommand{
    // EXIT,
    NONE,
    START,      // Bounding_CoM
    R2_A,       // bounding_slip
    L2_A,       // fixedStand
    L2_B,       // passive
    L2_X,       // freeStand
    L1_B,       // bounding_lip
#ifdef COMPILE_WITH_MOVE_BASE
    L2_Y,       // move_base
#endif  // COMPILE_WITH_MOVE_BASE
    L1_X,       // Bounding_Real_LIP
    R2_B,       // Bounding_Real_SLIP
    L1_A,       // swingTest
    L1_Y        // stepTest
};

enum class FrameType{
    BODY,
    HIP,
    GLOBAL
};

enum class WaveStatus{
    STANCE_ALL,
    SWING_ALL,
    WAVE_ALL
};

enum class FSMMode{
    NORMAL,
    CHANGE
};

enum class FSMStateName{
    // EXIT,
    INVALID,
    PASSIVE,
    FIXEDSTAND,
    FREESTAND,
    BOUNDING_COM,
    BOUNDING_SLIP,
    BOUNDING_LIP,
#ifdef COMPILE_WITH_MOVE_BASE
    MOVE_BASE,       // move_base
#endif  // COMPILE_WITH_MOVE_BASE
    BOUNDING_REAL_LIP,
    BOUNDING_REAL_SLIP,
    SWINGTEST,
    STEPTEST
};

#endif  // ENUMCLASS_H