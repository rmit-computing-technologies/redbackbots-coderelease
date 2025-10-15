#pragma once

#include <vector>

#include "perception/kinematics/RobotPose.hpp"
#include "types/BallInfo.hpp"
#include "types/field/FieldBoundary.hpp"
#include "types/FieldFeatureInfo.hpp"
#include "types/RobotVisionInfo.hpp"
#include "types/vision/RefereeGesture.hpp"

class VisionInfoOut {
public:
    VisionInfoOut() = default;

    std::array<FieldBoundary, CameraInfo::Camera::NUM_CAMERAS> fieldBoundary;
    std::vector<BallInfo> balls;
    std::vector<FieldFeatureInfo> features;
    std::vector<RobotVisionInfo> robots;

    // Referee
    RefereeGesture refereeGesture; // top camera only

    // Pose of the robot used in the vision calculation
    RobotPose kinematicPose;

    // OLD VISION OUT
    // int topStartScanCoords[TOP_IMAGE_COLS];
    // int botStartScanCoords[BOT_IMAGE_COLS];
    // std::vector<RegionI> regions;
};
