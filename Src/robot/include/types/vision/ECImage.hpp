/**
 * @file ECImage.h
 * 
 * Declares a representation containing both a color classified and a grayscale
 * version of the camera image.
 * It is advised to use this representation for all further image processing.
 *
 * @author Felix Thielke
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author RedbackBots
 */
 
#pragma once

#include "types/Image.hpp"
#include "types/PixelTypes.hpp"

/**
 * A representation containing both a color classified and a grayscale version of
 * the camera image.
 * It is advised to use this representation for all further image processing.
 */
class ECImage {
public:
  unsigned int timestamp = 0;
  Image<PixelTypes::GrayscaledPixel> grayscaled;
  Image<PixelTypes::GrayscaledPixel> saturated;
  Image<PixelTypes::HuePixel> hued;
  Image<PixelTypes::GrayscaledPixel> blueChromaticity;
  Image<PixelTypes::GrayscaledPixel> redChromaticity;
};

/**
 * A wrapper representation for the ECImage that allows not to store one.
 * This allows to stream an ECImage only if it is necessary.
 */
// class OptionalECImage {
//   public:
//     std::optional<ECImage> image;
// };

