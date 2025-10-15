/**
 * @file LineAndCircleMiddleInfo.hpp
 *
 * Declares a module which detects lines and the center circle based on ColorScanLineRegions.
 *
 * @author Felix Thielke
 * @author Lukas Monnerjahn
 * @author RedbackBots
 */

#pragma once

#include "perception/vision/Detector.hpp"
#include "blackboard/Blackboard.hpp"

#include "types/camera/CameraInfo.hpp"
#include "types/vision/ECImage.hpp"
#include "types/vision/RelativeFieldColors.hpp"
#include "types/vision/ColorScanLineRegions.hpp"
#include "types/vision/LineSpots.hpp"
#include "types/vision/CentreCircle.hpp"
#include "types/field/FieldBoundary.hpp"
#include "types/vision/Candidates.hpp"
#include "types/vision/CircleCandidates.hpp"
#include "types/vision/IsWhiteSpots.hpp"
#include "types/vision/RobotObstaclesImage.hpp"
#include "types/math/Eigen.hpp"
#include "utils/math/basic_maths.hpp"
#include "utils/debug/Assert.hpp"
#include "utils/math/LeastSquares.hpp"
#include "utils/defs/FieldDefinitions.hpp"

#include <limits>
#include <vector>

class LineAndCircleMiddleInfo : public Detector {
public:

	LineAndCircleMiddleInfo(Blackboard* blackboard);
  	virtual ~LineAndCircleMiddleInfo();

	// Resets middle info
	virtual void resetMiddleInfo(VisionInfoMiddle* info_middle);
	virtual void resetVisionOut(VisionInfoOut* info_out);

protected:
  	virtual void detect_(const VisionInfoIn* info_in, 
                      	VisionInfoMiddle* info_middle,
                      	VisionInfoOut* info_out);
private:

    std::vector<std::vector<Spot>> spotsH;
    std::vector<std::vector<Spot>> spotsV;
    std::vector<Candidate> candidates;
    std::vector<CircleCandidate, Eigen::aligned_allocator<CircleCandidate>> circleCandidates;

    /** distance in mm where the field next to the line is sampled during white checks */
    const float whiteCheckDistance = FIELD_LINE_WIDTH * 2;
    /** squared maximum line width in mm that can occur when fitting circle candidates to the image */
    const float circleCorrectionMaxLineWidthSquared = SQUARE(FIELD_LINE_WIDTH * 2);

    /**
     * Updates the LineSpots for the current frame.
     *
     * @param lineSpots the LineSpots to update
     * @param whichCamera
     * @param info_in
     * @param info_middle
     * @param info_out
     */
    void updateLineSpots(
        LineSpots& lineSpots, CameraInfo::Camera whichCamera, const VisionInfoIn* info_in, VisionInfoMiddle* info_middle,
        VisionInfoOut* info_out, Candidates& debugCandidates, Candidates& candidatesBefore,
        Candidates& candidatesAfter, CircleCandidates& debugCircleCandidates, IsWhiteSpots& isWhiteSpots);

    /**
     * Updates the CentreCircle for the current frame.
     *
     * @param centreCircle the CentreCircle to update
     * @param lineSpots the updated LineSpots
     * @param whichCamera
     * @param info_in
     * @param info_middle
     * @param info_out
     */
    void updateCentreCircle(CentreCircle& centreCircle, LineSpots& lineSpots, CameraInfo::Camera whichCamera,
        const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out,
        IsWhiteSpots& isWhiteSpots);

    /**
     * Scans the ColorScanLineRegionsHorizontal for line candidates.
     *
     * @param lineSpots representation in which the found lines are stored
     * @param info_in
     * @param cameraInfo CameraInfo passed through info_in
     * @param fieldBoundary FieldBoundary passed through info_middle
     * @param colorScanLineRegionsHorizontal ColorScanLineRegionsHorizontal passed through info_middle
     * @param ecImage ECImage passed through info_middle
     * @param relativeFieldColors RelativeFieldColors passed through info_middle
     */
    void scanHorizontalScanLines(LineSpots& lineSpots, 
        const VisionInfoIn* info_in, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary, ColorScanLineRegionsHorizontal& colorScanLineRegionsHorizontal,
        ECImage& ecImage, RelativeFieldColors& relativeFieldColors, Candidates& debugCandidates, Candidates& candidatesBefore,
        Candidates& candidatesAfter, IsWhiteSpots& isWhiteSpots, RobotObstaclesImage& robotsImage);

    /**
     * Scans the ColorScanLineRegionsVerticalClipped for line candidates.
     *
     * @param lineSpots representation in which the found lines are stored
     * @param info_in
     * @param cameraInfo CameraInfo passed through info_in
     * @param colorScanLineRegionsVerticalClipped ColorScanLineRegionsVerticalClipped passed through info_middle
     * @param ecImage ECImage passed through info_middle
     * @param relativeFieldColors RelativeFieldColors passed through info_middle
     */
    void scanVerticalScanLines(LineSpots& lineSpots, 
        const VisionInfoIn* info_in, const CameraInfo& cameraInfo, ColorScanLineRegionsVerticalClipped& colorScanLineRegionsVerticalClipped,
        ECImage& ecImage, RelativeFieldColors& relativeFieldColors, Candidates& debugCandidates,
        Candidates& candidatesBefore, Candidates& candidatesAfter, IsWhiteSpots& isWhiteSpots, RobotObstaclesImage& robotsImage);

    /**
     * Extends all found lines by tracing white lines in the image starting at
     * the line ends.
     *
     * @param lineSpots representation in which the lines to extend are stored
     * @param info_in
     * @param cameraInfo CameraInfo passed through info_in
     * @param ecImage ECImage passed through info_middle
     * @param relativeFieldColors RelativeFieldColors passed through info_middle
     */
    void extendLines(LineSpots& lineSpots, const VisionInfoIn* info_in, const CameraInfo& cameraInfo,
        ECImage& ecImage, RelativeFieldColors& relativeFieldColors, IsWhiteSpots& isWhiteSpots) const;

    /**
     * Checks whether the given line is extended by robots.
     * If this is the case, the line is trimmed accordingly.
     *
     * @param line the line to trim
     * @param info_in the line to trim
     * @param cameraInfo CameraInfo passed through info_in
     * @param ecImage ECImage passed through info_middle
     * @param relativeFieldColors RelativeFieldColors passed through info_middle
     */
    void trimLine(LineSpots::Line& line, const VisionInfoIn* info_in, const CameraInfo& cameraInfo, ECImage& ecImage, RelativeFieldColors& relativeFieldColors) const;

    /**
     * Checks whether a given segment can be added to a line candidate
     * @param a line spot of the segments first point
     * @param b line spot of the segments last point
     * @param candidate the line candidate
     * @param info_in
     * @param cameraInfo CameraInfo passed through info_in
     * @param ecImage ECImage passed through info_middle
     * @param relativeFieldColors RelativeFieldColors passed through info_middle
     * @param isWhiteSpots Debugging
     */
    bool isSegmentValid(const Spot& a, const Spot& b, const Candidate& candidate, 
        const VisionInfoIn* info_in, const CameraInfo& cameraInfo, ECImage& ecImage, RelativeFieldColors& relativeFieldColors,
        IsWhiteSpots& IsWhiteSpots);

    /**
     * Checks whether the given points are connected by a white line.
     *
     * @param a line spot of the first point
     * @param b line spot of the second point
     * @param n0Field normal on field of the line segment or the candidate it shall be added to
     * @param info_in
     * @param cameraInfo CameraInfo passed through info_in
     * @param ecImage ECImage passed through info_middle
     * @param relativeFieldColors RelativeFieldColors passed through info_middle
     */
    bool isWhite(const Spot& a, const Spot& b, const Vector2f& n0Field, 
        const VisionInfoIn* info_in, const CameraInfo& cameraInfo, ECImage& ecImage,
        RelativeFieldColors& relativeFieldColors, IsWhiteSpots& isWhiteSpots) /*const*/;

    /**
     * Determines the width of a line in the image with normal n0 at the given
     * spot in field coordinates.
     *
     * @param spot the spot to check
     * @param n0 normal of the line in field coordinates
     * @return the line width in field coordinates
     */
    float getLineWidthAtSpot(const Spot& spot, const Vector2f& n0) const;

    /**
     * Corrects the given center circle candidate by projecting spots on the
     * circle back into the image, shifting them to actual white spots in the
     * image and calculating a new circle in field candidates from the results.
     *
     * @param circle circle candidate to correct
     * @param info_in circle candidate to correct
     * @param cameraInfo CameraInfo passed through info_in
     * @param fieldBoundary FieldBoundary passed through info_out
     * @param ecImage ECImage passed through info_middle
     * @param relativeFieldColors RelativeFieldColors passed through info_middle
     * @return whether the candidate is valid before and after the correction
     */
    bool correctCircle(CircleCandidate& circle, 
        const VisionInfoIn* info_in, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary,
        ECImage& ecImage, RelativeFieldColors& relativeFieldColors, IsWhiteSpots& isWhiteSpots) const;

    /**
     * Sets a flag on all lines in the LinePercept that lie on the detected center
     * circle.
     *
     * @param center center of the detected circle
     * @param lineSpots
     */
    void markLinesOnCircle(const Vector2f& center, LineSpots& lineSpots);

    /**
     * Checks if the model error of the circle is lower than the error if the spots
     * were fitted to a line.
     *
     * @param circle the circle candidate to check
     * @param lineError returns the model error of the fitted line
     * @return true if the circle error is lower than the line error
     */
    bool isCircleNotALine(const CircleCandidate& circle, float& lineError) const;

    /**
     * Checks whether the given center circle candidate is white when projected
     * into image coordinates.
     *
     * @param center center of the circle in field coordinates
     * @param radius radius of the circle in field coordinates
     * @param info_in circle candidate to correct
     * @param cameraInfo CameraInfo passed through info_in
     * @param ecImage ECImage passed through info_middle
     * @param relativeFieldColors RelativeFieldColors passed through info_middle
     * @return result
     */
    bool isCircleWhite(const Vector2f& center, const float radius,
        const VisionInfoIn* info_in, const CameraInfo& cameraInfo, ECImage& ecImage,
        RelativeFieldColors& relativeFieldColors, IsWhiteSpots& isWhiteSpots) const;

    /**
     * Tests whether a given point qualifies as white based on reference points
     * on both sides of the line or circle it lies on.
     * @param pointOnField checked point in field coordinates
     * @param pointInImage checked point in image coordinates
     * @param n0 normal vector of the checked line point i.e. direction in which to expect field
     * @param info_in circle candidate to correct
     * @param cameraInfo CameraInfo passed through info_in
     * @param ecImage ECImage passed through info_middle
     * @param relativeFieldColors RelativeFieldColors passed through info_middle
     * @return true, if the point is considered white
     */
    bool isPointWhite(const Vector2f& pointOnField, const Vector2i& pointInImage, const Vector2f& n0,
        const VisionInfoIn* info_in, const CameraInfo& cameraInfo, ECImage& ecImage, RelativeFieldColors& relativeFieldColors,
        IsWhiteSpots& isWhiteSpots) const;

    /**
     * Tests whether a given point qualifies as white based on reference points
     * on both sides of the line or circle it lies on.
     * @param pointInImage checked point in image coordinates
     * @param n points from the pointInImage to the reference points
     * @param cameraInfo CameraInfo passed through info_in
     * @param ecImage ECImage passed through info_middle
     * @param relativeFieldColors RelativeFieldColors passed through info_middle
     * @return true, if the point is considered white
     */
    bool isPointWhite(const Vector2f& pointInImage, const Vector2f& n,
        const CameraInfo& cameraInfo, ECImage& ecImage, RelativeFieldColors& relativeFieldColors, IsWhiteSpots& isWhiteSpots) const;

    /**
     * Calculates the on field distance that a reference point for a white check should have to a given point
     * @param pointOnField the point
     * @return the points white check distance on field
     */
    float calcWhiteCheckDistance(const Vector2f& pointOnField) const;

    /**
     * Calculates the pixel distance a reference point for a white check should have to a given point.
     * @param pointOnField the Point in field coordinates. Used to estimate its distance
     * @param cameraInfo CameraInfo passed through info_in
     * @return the points white check distance in pixels
     */
    float calcWhiteCheckDistanceInImage(const Vector2f& pointOnField, const CameraInfo& cameraInfo) const;

    /**
     * Finds the vertical scan line straight left of the given spot.
     * This handles the possibly differing vertical scan line lengths
     * @param spot A spot on a vertical scan line
     * @param scanLineId id of the scanline the spot lies on
     * @param colorScanLineRegionsVerticalClipped ColorScanLineRegionsVerticalClipped passed through info_middle
     * @return id of the scan line left of the spot
     */
    int previousVerticalScanLine(const Spot& spot, int scanLineId,
        ColorScanLineRegionsVerticalClipped& colorScanLineRegionsVerticalClipped) const;

    /**
     * Checks whether the given region lies within an obstacle in the ObstaclesPercept.
     *
     * @param fromX left coordinate of the region
     * @param toX right coordinate of the region
     * @param fromY upper coordinate of the region
     * @param toY lower coordinate of the region
     */
    bool isSpotInsideObstacle(RobotObstaclesImage& robotsImage, const int fromX, const int toX, const int fromY, const int toY) const;

    bool debugIsPointWhite = false;

	void configure(Blackboard* blackboard);

	void detect_(CameraInfo::Camera whichCamera,
                const VisionInfoIn* info_in, 
                VisionInfoMiddle* info_middle,
                VisionInfoOut* info_out);

    int maxLineWidthDeviationPx = 20; /**< maximum deviation of line width in the image to the expected width at that position in px */
    int maxSkipWidth = 2; /**< regions with a size of up to this many pixels can be skipped next to lines. */
    int maxSkipNumber = 2; /**< The maximum number of neighboring regions to skip. */
    float greenAroundLineRatio = 2.5; /**< minimum green next to the line required as factor of line width. */
    float greenAroundLineRatioCalibration = 2.5;
    float maxDistantHorizontalLength = 600; /**< maximum length of distant horizontal lines in mm */
    float maxLineFittingError = 50; /**< maximum error of fitted lines through spots on the field in mm */
    unsigned minSpotsPerLine = 4; /**< minimum number of spots per line */
    unsigned minSpotsPerLineCalibration = 3;
    unsigned whiteCheckStepSize = 20; /**< step size in px when checking if lines are white */
    float minWhiteRatio = 0.75; /**< minimum ratio of white pixels in lines */
    float minSquaredLineLength = 22500; /**< minimum squared length of found lines in mm */
    float maxCircleFittingError = 100; /**< maximum error of fitted circles through spots on the field in mm */
    float maxCircleRadiusDeviation = 100; /**< maximum deviation in mm of the perceived center circle radius to the expected radius */
    Angle minCircleAngleBetweenSpots = 45_deg; /**< minimum angular distance around the circle center that has to be spanned by a circle candidate */
    unsigned int minSpotsOnCircle = 10; /**< minimum number of spots on the center circle */
    float minCircleWhiteRatio = 0.75; /**< minimum ratio of white pixels in the center circle */
    bool trimLines = true; /**< whether lines extended by robots shall be trimmed */
    bool trimLinesCalibration = false;
    int maxWidthImage = 20; /**< maximum width of a line in the image at a spot up to which the spot is considered a valid line spot without further checks*/
    float maxWidthImageSquared = 400; /**< maximum squared width of a line in the image at a spot up to which the spot is considered a valid line spot without further checks */
    float mFactor = 2; /**< the calculated width of a line at a spot in mm must be below the expected width multiplied by this factor to consider the spot a valid line spot */
    int minConsecutiveSpots = 5; /**< number of consecutive valid line spots found at which trimming shall be stopped */

    float squaredWhiteCheckNearField = 4410000; /**< squared distance in mm from which on the whiteCheckDistance becomes increased due to possibly blurry images */
    float maxNormalAngleDiff = 0.785398163; /**< maximum angle difference between two normal vectors for two line segments possibly belong to the same line */
    bool relaxedGreenCheckAtImageBorder = true; /**< accept line spots with too small neighboring green regions if they reach to the image border */
    bool perspectivelyCorrectWhiteCheck = true; /**< true: transform image to field, compute test points on field, project back; false: compute test points in image */
    bool highResolutionScan = true; /**< use all the vertical scan lines or only the low resolution subset */
};