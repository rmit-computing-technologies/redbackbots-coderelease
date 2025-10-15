#pragma once

#include "types/math/Geometry.hpp"
#include <vector>

/**
 * @struct LineSpots
 * This struct contains all the linespots
 */
class LineSpots {
public:
    struct Line {
        Line() = default;
        Line(const Vector2f& base, const Vector2f& direction) : line(base, direction) {};

        Geometry::Line line; /**<The fitted line in field coordinates */
        std::vector<Vector2f> spotsInField; /**< Spots that are on this line  (in relative field coordinates). NOT FITTED TO LINE*/
        std::vector<Vector2i> spotsInImg; /**< Spots that are on this line  (in image coordinates). NOT FITTED TO LINE*/
        Vector2i firstImg; /**<First spot of the line in image coordinates, NOT FITTED TO LINE*/
        Vector2i lastImg; /**<Last spot of the line in image coordinates, NOT FITTED TO LINE*/
        Vector2f firstField; /**<start of the fitted line in field coordinates */
        Vector2f lastField; /**< end of the fitted line in field coordinates */
        bool belongsToCircle = false;
        float length;
    };

    std::vector<Line> lines;
};