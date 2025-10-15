#pragma once

#include "perception/kinematics/RobotPose.hpp"
#include "types/camera/CameraImage.hpp"
#include "types/camera/CameraInfo.hpp"
#include "types/camera/CameraResolution.hpp"
#include "types/camera/CameraSettings.hpp"
#include "types/ActionCommand.hpp"
#include "types/geometry/AbsCoord.hpp"

#include <array>

class VisionInfoIn {
public:
    VisionInfoIn() = default;

    // Camera Image
    std::array<const CameraImage*, CameraInfo::Camera::NUM_CAMERAS> image;

    // Camera Properties
    std::array<CameraInfo, CameraInfo::Camera::NUM_CAMERAS> cameraInfo;
    std::array<CameraResolution, CameraInfo::Camera::NUM_CAMERAS> cameraResolution;
    std::array<CameraSettings, CameraInfo::Camera::NUM_CAMERAS> cameraSettings;

    // NEED TO MAKE SURE THIS IS SET EACH FRAME AS IT WAS IN THE OLD SYSTEM.
    // This accounts for leaning that is not properly considered by
    // imageToRobotXY. imageToRobotXY either doesn't account for leaning of the
    // whole robot OR doesn't account for some kind of delay in gyro reading.
    float latestAngleX;

    // Pose of the robot computed from motion.
    // Only gives robot-relative transforms from the robot kinematics
    RobotPose kinematicPose;

    // World Pose of robot from State Estimation
    AbsCoord worldPose;

    // Player Number to conditionally change feature detection
    int playerNum;

    // Store Sensor values
    // Left out for now, unless these are required later
    // SensorValues sensorValues; 
};
