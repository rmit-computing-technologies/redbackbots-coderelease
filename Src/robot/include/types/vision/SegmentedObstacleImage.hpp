/**
 * @file SegmentedObstacleImage.hpp
 *
 * Declares a representation containing a downscaled obstacle segmented version of the camera image.
 *
 * @author Laurens Müller-Groh
 * @author RedbackBots
 */

#pragma once

#include "types/Image.hpp"
#include "types/PixelTypes.hpp"

class SegmentedObstacleImage {
public:
    Image<PixelTypes::GrayscaledPixel> obstacle;

};