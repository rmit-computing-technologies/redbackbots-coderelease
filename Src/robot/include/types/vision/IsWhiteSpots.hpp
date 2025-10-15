#pragma once

#include "Candidates.hpp"
#include <vector>

/**
 * @struct IsWhiteSpots
 * This struct contains the reference points for all is white checks
 */

struct Comparison {
    Spot pointInImage;
    Spot referencePoint;
    int type;
    Comparison(Spot pointInImage, Spot referencePoint, int type) : pointInImage(pointInImage), referencePoint(referencePoint), type(type) {}
    Comparison() {}

    bool hasZero() {
        if (
            (pointInImage.image.x() <= 0 || pointInImage.image.y() <= 0) ||
            (referencePoint.image.x() <= 0 || referencePoint.image.y() <= 0)
        ) {
            return true;
        }
        else {
            return false;
        }
    }
};

class IsWhiteSpots {
    public:
        std::vector<Comparison> comparisons;
};