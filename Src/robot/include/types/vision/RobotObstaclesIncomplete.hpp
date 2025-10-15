/**
 * @file RobotObstaclesIncomplete.hpp
 *
 * This file defines a representation that the ObstaclePerceptor sends from its
 * upper image to its lower image instance.
 *
 * @author Thomas Röfer
 * @author RedbackBots
 */

#pragma once

#include "types/vision/RobotObstaclesField.hpp"
#include "types/camera/CameraInfo.hpp"

class RobotObstaclesIncomplete {
public:
  struct ScanScore {
    ScanScore() = default;

    /**
     * Constructor.
     * @param shortRangeScore The short range score at the lower end of the scan.
     * @param longRangeScore The long range score at lower end of the scan.
     */
    ScanScore(int shortRangeScore, int longRangeScore);

    int shortRangeScore; /**< The short range score at the lower end of the scan. */
    int longRangeScore; /**< The long range score at lower end of the scan. */
  };
  
  std::vector<ScanScore> scanScores;
  std::vector<RobotObstaclesField::Obstacle> incompleteObstacles;

};

inline RobotObstaclesIncomplete::ScanScore::ScanScore(int shortRangeScore, int longRangeScore)
  : shortRangeScore(shortRangeScore), longRangeScore(longRangeScore) {};
