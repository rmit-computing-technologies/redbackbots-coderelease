#pragma once

/**
 * @file VisionDebuggerBlackboard.hpp
 * 
 * Vision debugging information, typically vision middle info data.
 */

#ifdef TMP_NDEBUG

// This case is not required for individual debugger blackboards.
// This is handled by the wrapper Debugger blackboard

#else

#include "types/camera/CameraInfo.hpp"
#include "types/vision/BallSpots.hpp"
#include "types/vision/ScanGrid.hpp"
#include "types/vision/ColorScanLineRegions.hpp"
#include "types/vision/PenaltyMarkRegions.hpp"
#include "types/vision/LineSpots.hpp"
#include "types/vision/Candidates.hpp"
#include "types/vision/CircleCandidates.hpp"
#include "types/vision/CentreCircle.hpp"
#include "types/vision/IntersectionCandidates.hpp"
#include "types/vision/RefereeKeypoints.hpp"
#include "types/vision/PlaneSpots.hpp"
#include "types/vision/IsWhiteSpots.hpp"
#include "types/vision/RobotObstaclesImage.hpp"

#include <array>

/**
 * Data for debugging vision.
 * 
 * Information here is only sent to offnao / vatnao.
 * This class is only compiled for Debug and Develop targets.
 * Otherwise for competition mode, this is 'compiled' out.
 */
class VisionDebuggerBlackboard {
public:
    explicit VisionDebuggerBlackboard();

    /** Scan Grid for both cameras */
    std::array<ScanGrid, CameraInfo::Camera::NUM_CAMERAS> scanGrid;

    /** Horizontal Scan Lines for both cameras */
    std::array<ColorScanLineRegionsHorizontal, CameraInfo::Camera::NUM_CAMERAS> colorScanLineRegionsHorizontal;

    /** Vertical Scan Lines for both cameras */
    std::array<ColorScanLineRegionsVerticalClipped, CameraInfo::Camera::NUM_CAMERAS> colorScanLineRegionsVerticalClipped;

    /** Penalty Mark Regions for both cameras */
    std::array<PenaltyMarkRegions, CameraInfo::Camera::NUM_CAMERAS> penaltyMarkRegions;

    /** Ball Spots (pre-cursor to ball detector) for both cameras */
    std::array<BallSpots, CameraInfo::Camera::NUM_CAMERAS> ballSpots;

    /** Line Spots (pre-cursor to Field Features) */
    std::array<LineSpots, CameraInfo::Camera::NUM_CAMERAS> lineSpots;

    /** Candidates (pre-cursor to LineSpots) */
    std::array<Candidates, CameraInfo::Camera::NUM_CAMERAS> candidates;

    /** LineSpots Candidates before removal */
    std::array<Candidates, CameraInfo::Camera::NUM_CAMERAS> candidatesBefore;

    /** LineSpots Candidates after removal */
    std::array<Candidates, CameraInfo::Camera::NUM_CAMERAS> candidatesAfter;

    /** Centre Circle Candidates */
    std::array<CircleCandidates, CameraInfo::Camera::NUM_CAMERAS> circleCandidates;

    /** Centre Circle */
    std::array<CentreCircle, CameraInfo::Camera::NUM_CAMERAS> centreCircle;

    /** Intersection Candidates (pre-cursor to intersections) */
    std::array<IntersectionCandidates, CameraInfo::Camera::NUM_CAMERAS> intersectionCandidates;

    /** Referee Keypoints */
    RefereeKeypoints refereeKeypoints;
    
    /** Plane Spots */
    std::array<PlaneSpots, CameraInfo::Camera::NUM_CAMERAS> planeSpots;

    /** isWhite Spots */
    std::array<IsWhiteSpots, CameraInfo::Camera::NUM_CAMERAS> isWhiteSpots;

    /** Ball Spots (pre-cursor to ball detector) for both cameras */
    std::array<RobotObstaclesImage, CameraInfo::Camera::NUM_CAMERAS> robotsImage;
};


#endif
