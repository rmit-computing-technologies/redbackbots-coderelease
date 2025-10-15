/**
 * @file RefereeKeypoints.hpp
 *
 * This file declares a representation that represents the output of the
 * keypoint detector, i.e. pixel positions and confidences of 17 different
 * body parts.
 *
 * @author Thomas Röfer
 * @author RedbackBots
*/

#pragma once

#include "types/math/Boundary.hpp"

#include <array>

class RefereeKeypoints {
public:
    enum Keypoint : int {
        nose,
        leftEye,
        rightEye,
        leftEar,
        rightEar,
        leftShoulder,
        rightShoulder,
        leftElbow,
        rightElbow,
        leftWrist,
        rightWrist,
        leftHip,
        rightHip,
        leftKnee,
        rightKnee,
        leftAnkle,
        rightAnkle,
        count
    };

    struct Point {
        Vector2f position;
        bool valid = false;
    };

    std::array<Point, Keypoint::count> points = {}; /**< The keypoints in image coordinates. */
    Boundaryi patchBoundary; /**< The boundary of the patch that was searched for the keypoints. */

    inline void reset() {
        for (Point point : points) {
            point.position = {0,0};
            point.valid = false;
        }
    };
};