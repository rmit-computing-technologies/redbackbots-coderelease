#pragma once

#include <array>
#include <vector>

#include "types/field/FieldFeatureLocations.hpp"
#include "types/vision/ScanGrid.hpp"
#include "types/vision/ECImage.hpp"
#include "types/vision/RelativeFieldColors.hpp"
#include "types/vision/ColorScanLineRegions.hpp"
#include "types/vision/PenaltyMarkRegions.hpp"
#include "types/vision/PenaltyMarkInfo.hpp"
#include "types/vision/LineSpots.hpp"
#include "types/vision/CentreCircle.hpp"
#include "types/vision/IntersectionCandidates.hpp"
#include "types/vision/Intersections.hpp"
#include "types/vision/BallSpots.hpp"
#include "types/vision/PreviousBalls.hpp"
#include "types/vision/RobotObstaclesField.hpp"
#include "types/vision/RobotObstaclesImage.hpp"
#include "types/vision/RobotObstaclesIncomplete.hpp"
#include "types/vision/SegmentedObstacleImage.hpp"
#include "types/vision/Candidates.hpp"
#include "types/vision/CircleCandidates.hpp"
#include "types/vision/RefereeKeypoints.hpp"
#include "types/vision/PlaneSpots.hpp"
#include "types/vision/IsWhiteSpots.hpp"

class VisionInfoMiddle {
public:
    VisionInfoMiddle() = default;

    // Static Field Dimensions - locations of field elements
    FieldFeatureLocations fieldFeatureLocations;

    // Various Scan Grid detail levels
    std::array<ScanGrid, CameraInfo::Camera::NUM_CAMERAS> scanGrid;

    // Various ECImage detail levels
    std::array<ECImage, CameraInfo::Camera::NUM_CAMERAS> ecImage;

    // Various RelativeFieldColors detail levels
    std::array<RelativeFieldColors, CameraInfo::Camera::NUM_CAMERAS> relativeFieldColors;
    // std::array<RelativeFieldColorsParameters, CameraInfo::Camera::NUM_CAMERAS> relativeFieldColorsParameters;

    // Various ColourScanLineRegion detail levels
    std::array<ColorScanLineRegionsHorizontal, CameraInfo::Camera::NUM_CAMERAS> colorScanLineRegionsHorizontal;
    std::array<ColorScanLineRegionsVerticalClipped, CameraInfo::Camera::NUM_CAMERAS> colorScanLineRegionsVerticalClipped;

    // Potential penalty mark regions
    std::array<PenaltyMarkRegions, CameraInfo::Camera::NUM_CAMERAS> penaltyMarkRegions;

    // Penalty mark details
    std::array<PenaltyMarkInfo, CameraInfo::Camera::NUM_CAMERAS> penaltyMarkInfo;

    // Various Field Line detail levels
    std::array<LineSpots, CameraInfo::Camera::NUM_CAMERAS> lineSpots;
    std::array<CentreCircle, CameraInfo::Camera::NUM_CAMERAS> centreCircle;
    std::array<Candidates, CameraInfo::Camera::NUM_CAMERAS> candidates; // LineSpot Candidates
    std::array<Candidates, CameraInfo::Camera::NUM_CAMERAS> candidatesBefore; // LineSpots Candidates before removal
    std::array<Candidates, CameraInfo::Camera::NUM_CAMERAS> candidatesAfter; // LineSpots Candidates after removal
    std::array<CircleCandidates, CameraInfo::Camera::NUM_CAMERAS> circleCandidates; // Candidates for centre circle

    // Various Intersection details
    std::array<IntersectionCandidates, CameraInfo::Camera::NUM_CAMERAS> intersectionCandidates;
    std::array<Intersections, CameraInfo::Camera::NUM_CAMERAS> intersections;

    // Various BallSpots details
    std::array<BallSpots, CameraInfo::Camera::NUM_CAMERAS> ballSpots;
    std::array<PreviousBalls, CameraInfo::Camera::NUM_CAMERAS> previousBalls;

    // Robot detection details
    std::array<RobotObstaclesField, CameraInfo::Camera::NUM_CAMERAS> robotsField;
    std::array<RobotObstaclesImage, CameraInfo::Camera::NUM_CAMERAS> robotsImage;
    std::array<RobotObstaclesIncomplete, CameraInfo::Camera::NUM_CAMERAS> obstaclesData;
    SegmentedObstacleImage segmentedObstacleImage;

    // Referee Keypoints
    RefereeKeypoints refereeKeypoints; // Only in top camera

    // Plane Spots
    std::array<PlaneSpots, CameraInfo::Camera::NUM_CAMERAS> planeSpots;

    // IsWhite Spots
    std::array<IsWhiteSpots, CameraInfo::Camera::NUM_CAMERAS> IsWhiteSpots;

    // OLD MIDDLE INFO
    // Regions covering the full frame.
    // std::vector<RegionI> full_regions;

    // Regions covering only key areas of interest in the frame.
    // std::vector<RegionI> roi;

    // Data created by field features associated with each ROI.
    // std::vector<FieldFeatureRegionData> fieldFeatureRegionData;

    // Field features data on other features.

    // All valid penalty cross
    // std::vector<Point> valid_penalty_cross;

    // All valid field lines.
    // std::vector<RANSACLine> valid_lines;

    // Buffered squared length of each line.
    // std::vector<int> line_lengths;

    // All valid centre circles.
    // std::vector<PointF, Eigen::aligned_allocator<PointF> > circle_centres;

    // The quality of each centre circle.
    // std::vector<float> circle_qualities;

    // The centre line associated with each centre circle.
    // std::vector<RANSACLine> circle_centre_lines;

    // The ID of the highest quality centre circle.
    // int best_centre_circle;

    // The number of non line features sent to localisation by field features.
    // int features_sent;

    // All intersections between near perpendicular lines.
    // std::vector<Point, Eigen::aligned_allocator<Point> > intersections;

    // All potential corner intersections
    // std::vector<Point, Eigen::aligned_allocator<Point> > potentialCornerIntersections;

    // std::vector<float> potentialCornerIntersectionAngles;

    // All T intersections of potential T junctions
    // std::vector<Point, Eigen::aligned_allocator<Point> > potentialTIntersections;

    // All angles of potential T junctions
    // std::vector<float> potentialTIntersectionAngles;

    // The IDs of the lines that form each intersection.
    // std::vector<std::pair<int, int> > intersection_lines;

    // Whether each intersection appears to be on each of its forming lines.
    // std::vector<std::pair<bool, bool> > on_lines;

    // The quality of each intersection.
    // std::vector<float> intersection_qualities;

    // Extra quality for an intersection to be marked as a corner.
    // std::vector<float> intersection_corner_qualities;

    // Extra quality for an intersection to be marked as a T.
    // std::vector<float> intersection_T_qualities;

    // Extra quality for an intersection to be marked as a centre circle 4 way
    // junction.
    // std::vector<float> intersection_4_way_qualities;

    // The raw data associated with this frame.
    // const CombinedFrame* this_frame;

    // std::vector<Point> basePoints;
    // std::vector<Point> basePointImageCoords;
};
