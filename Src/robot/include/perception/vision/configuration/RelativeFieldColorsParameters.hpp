/**
 * @file RelativeFieldColorsParameters.h
 *
 * This file declares a representation that contains common thresholds
 * for distinguishing colors of neighboring regions/pixels.
 *
 * @author Arne Hasselbring
 * @author RedbackBots
 */

#pragma once

#include "types/math/Range.hpp"

class RelativeFieldColorsParameters {
public:
  unsigned char minWhiteToFieldLuminanceDifference = 15;  /**< White and field color differ at least this much in luminance if they are next to each other in an image. */
  unsigned char minWhiteToFieldSaturationDifference = 15; /**< White and field color differ at least this much in saturation if they are next to each other in an image. */
  unsigned char minFieldSaturation = 35;  /**< No field pixel is ever less saturated than this. */
  unsigned char  maxWhiteSaturation = 100; /**< No white pixel is ever more saturated than this. */
  unsigned char  maxFieldLuminance = 200;  /**< No field pixel is ever lighter than this. */
  unsigned char minWhiteLuminance = 70;   /**< No white pixel is ever darker than this. */
  Rangeuc fieldHue = {100, 180}; /**< A hue range which is guaranteed to contain the field color. */
};
