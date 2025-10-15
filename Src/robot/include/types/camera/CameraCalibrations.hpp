/**
 * @file CameraIntrinsics.hpp
 * Declaration of a struct for representing the intrinsic parameters of the cameras.
 * @author Alexis Tsogias
 * @author Redbackbots
 */

#pragma once

#include "types/math/Angle.hpp"
#include "types/math/Eigen.hpp"

/**
 * Struct representing calibrated camera offsets 
 */
struct CameraRotationCorrections {
    float x = 0; /**< X-axis rotation correction in radians */
    float y = 0; /**< Y-axis rotation correction in radians */
    float z = 0; /**< Z-axis rotation correction in radians */

    CameraRotationCorrections() = default;

    CameraRotationCorrections(float x, float y, float z) : x(x), y(y), z(z) {}
};

class CameraCalibrations {
public:
    CameraRotationCorrections cameraRotationCorrections; /**< Camera rotation corrections for the camera */
};
