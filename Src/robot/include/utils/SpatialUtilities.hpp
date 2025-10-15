/**
 * @file SpatialUtilities.hpp
 * 
 * Utilities for common checks or operations for Spatial or Geometric
 * computations, such as field boundary checks.
 * 
 * @author RedbackBots
 */

#pragma once

#include "types/geometry/Point.hpp"
#include "utils/defs/FieldDefinitions.hpp"

#include <cmath>

class SpatialUtilities {
public:

    /**
     * Check if a given Robot Relative XY (ground-plane) point could possibly
     * reside on the SPL Field. Checks with the maximum Field Dimension
     */
    static inline bool possiblyOnFieldRRXY(const PointF& point) {
        return std::abs(point.x()) < MAX_DIST_ON_FIELD
               && std::abs(point.y()) < MAX_DIST_ON_FIELD;
    };
};
