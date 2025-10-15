#pragma once

#include "perception/kinematics/RobotPose.hpp"
#include "types/ActionCommand.hpp"
#include "types/ButtonPresses.hpp"
#include "types/JointValues.hpp"
#include "types/MotionDebugInfo.hpp"
#include "types/Odometry.hpp"
#include "types/SensorValues.hpp"
#include "types/XYZ_Coord.hpp"

/**
 * Shared data from the motion module.
 */
struct MotionBlackboard {
    explicit MotionBlackboard();

    SensorValues sensors;
    float uptime;
    ActionCommand::All active;
    Odometry odometry;
    ButtonPresses buttons;
    RobotPose pose;
    XYZ_Coord com;
    JointValues jointRequest;
    MotionDebugInfo motionDebugInfo;
};
