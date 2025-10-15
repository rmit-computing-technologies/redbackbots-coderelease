/**
 * @file CentreCircle.h
 * Declaration of a struct that represents the center circle.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author RedbackBots
 */

#pragma once

#include "types/math/Eigen.hpp"

/**
 * @struct CentreCircle
 * A struct that represents the found field center circle.
 */
class CentreCircle {
public:
    Vector2f pos = Vector2f::Zero(); /**< The position of the center of the center circle in field coordinates */
    Vector2i image = Vector2i::Zero(); /**< The position of the center of the center circle in image coordinates */
    // Matrix2f cov; /**< The covariance of the center circle "measurement" */
    bool wasSeen = false; /**< Has the percept been seen in the last frame? */

    inline void reset() {
        pos = Vector2f::Zero();
        image = Vector2i::Zero();
        // cov = Matrix2f::Zero();
        wasSeen = false;
    }
};