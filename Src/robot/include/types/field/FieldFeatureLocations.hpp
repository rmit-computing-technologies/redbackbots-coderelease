
/**
 * @file FieldFeatureLocations.hpp
 * 
 * Defines static positions of field features.
 * Generally used by localisation, but also available to other
 * 
 */

#pragma once

#include "types/field/FieldFeature.hpp"
#include "types/math/Boundary.hpp"

#include <vector>

class FieldFeatureLocations {
public:

    FieldFeatureLocations();

    std::vector<FieldFeature> corners;
    std::vector<FieldFeature> t_junctions;
    std::vector<FieldFeature> x_junctions;
    std::vector<FieldFeature> centre_circles;
    std::vector<FieldFeature> penalty_spots;
    std::vector<float> constantXLines;
    std::vector<float> constantYLines;

    // Edge of the green carpet, in X/Y ranges
    Boundaryf edge;
};

