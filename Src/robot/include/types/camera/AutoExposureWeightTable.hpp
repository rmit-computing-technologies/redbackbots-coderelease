/**
 * @file AutoExposureWeightTable.h
 *
 * This file implements a representation to describe how the camera 
 * is using the image regions to calculate the auto exposure.
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Redbackbots
 */

#pragma once

#include <array>
#include "types/camera/CameraInfo.hpp"
#include "types/math/Eigen.hpp"

class AutoExposureWeightTable {
public:
  static constexpr int width = 4;
  static constexpr int height = 4;
  static constexpr uint8_t maxWeight = 15;

  // Cells are allowed to have values from 0 to maxWeight (inclusive)
  using Table = Eigen::Matrix<uint8_t, height, width, Eigen::RowMajor>;
};
