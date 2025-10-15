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
 * This struct stores the opening angles and optical centers of both cameras.
 * The optical center is stored in as a ratio of coordinates with a range from 0 to 1.
 * That is, 0.5 (default config) is a ratio of the optical centre being 50% of the width/height
 * Computing the optical center in pixels is a matter of multiplying ratio with the width / height of the cameras.
 * Thus, only resolutions with the same aspect ratios should be used.
 */
class CameraIntrinsics {
public:
    Angle openingAngleWidth;
    Angle openingAngleHeight;
    Vector2f opticalCenter;
};
