/**
 * @file PenaltyMarkInfo.hpp
 * Declaration of a struct that represents a penalty mark.
 * @author Maik Schünemann
 * @author RedbackBots
 */

#pragma once

#include "types/math/Eigen.hpp"

/**
 * @struct PenaltyMarkInfo
 * A struct that represents a detected penalty mark.
 */
struct PenaltyMarkInfo
{
  Vector2i positionInImage = Vector2i::Zero(); /**< Position in the image. */
  Vector2f positionOnField = Vector2f::Zero(); /**< Position relative to robot on the field. */
  // Matrix2f covarianceOnField; /**< The measurement covariance of positionOnField */
  bool wasSeen = false;  /**< Indicates whether the feature has been seen in the last frame */

  void reset(){
    positionInImage = Vector2i::Zero();
    positionOnField = Vector2f::Zero();
    wasSeen = false;
  }
};
