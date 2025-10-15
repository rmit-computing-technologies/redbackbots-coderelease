/**
 * @file types/field/FieldBoundary.hpp
 * 
 * Defines the boundary of the field in both Image and Field coordinates
 * The boundary is a vector of points (not a line straight line)
 * 
 * @author Alexis Tsogias
 * @author RedbackBots
 */

#pragma once

#include "types/math/Eigen.hpp"

#include <vector>


class FieldBoundary {
public:
    FieldBoundary();
    ~FieldBoundary() {}

    /**
     * Returns the y coordinate of the field boundary at the specified x coordinate in the current image.
     */
    int getBoundaryY(int x) const;

    /**
     * @param imageWidth The width of the current image.
     * @return The topmost y coordinate of the field boundary in the current image.
     */
    int getBoundaryTopmostY(int imageWidth) const;

    // Clear and reset the contents of the Field Boundary
    void reset();

    // The boundary projected to the field in relative coordinates.
    std::vector<Vector2f> boundaryOnField;
    
    // The boundary in image coordinates.
    std::vector<Vector2i> boundaryInImage;

    // Boundary in image with uncertainty
    std::vector<Vector2i> boundaryInImageLowerBound;
    std::vector<Vector2i> boundaryInImageUpperBound;

    // True if a boundary could be detected.
    bool isValid;

    // True if the boundary is constructed from a previous field boundary
    bool extrapolated;

    // True if the boundary is odd/not good
    bool odd;
};

