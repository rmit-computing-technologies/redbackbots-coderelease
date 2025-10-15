/**
 * @file FieldFeatureDetector.hpp
 * 
 * @author Arne Böckmann
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author RedbackBots
 */

#pragma once

#include "perception/vision/Detector.hpp"
#include "blackboard/Blackboard.hpp"

#include "utils/defs/BallDefinitions.hpp"
#include "utils/defs/FieldDefinitions.hpp"
#include "types/camera/CameraImage.hpp"
#include "types/camera/CameraInfo.hpp"
#include "types/vision/CentreCircle.hpp"
#include "types/vision/Intersections.hpp"
#include "types/vision/LineSpots.hpp"
#include "types/vision/PenaltyMarkInfo.hpp"
#include "types/math/Eigen.hpp"
#include "types/geometry/RRCoord.hpp"
#include "utils/debug/Assert.hpp"

class FieldFeatureDetector : public Detector {
public:

	FieldFeatureDetector(Blackboard* blackboard);
  	virtual ~FieldFeatureDetector();

	// Resets middle info
	virtual void resetMiddleInfo(VisionInfoMiddle* info_middle);
	virtual void resetVisionOut(VisionInfoOut* info_out);

protected:
  	virtual void detect_(const VisionInfoIn* info_in, 
                      	VisionInfoMiddle* info_middle,
                      	VisionInfoOut* info_out);
private:
    enum SpotLineStatus {
        thrown,
        stayed,
    };

    std::vector<SpotLineStatus> spotLineUsage;
    CentreCircle lastCentreCircle;
    // Odometry lastOdometry;
    unsigned int lastFrameTime = 1;
    std::vector<LineSpots::Line> internalListOfLines;  /**< Unsorted list of computed field lines. */

    LineSpots::Line* midLine;

    bool isPointInSegment(const LineSpots::Line& line, const Vector2f& point) const;

    std::vector<size_t> processLines(const VisionInfoIn* info_in, const CameraImage* cameraImage, const CameraInfo& cameraInfo, LineSpots& lineSpots, CentreCircle& centreCircle, std::vector<FieldFeatureInfo>& features);

    void updateLines(const CameraInfo& cameraInfo, LineSpots& lineSpots, std::vector<size_t> sortedLineIndizes, std::vector<FieldFeatureInfo>& features, int playerNum);

    void updateCentreCircle(const VisionInfoIn* info_in, const CameraInfo& cameraInfo, CentreCircle& centreCircle, std::vector<FieldFeatureInfo>& features);

    void updateIntersections(const VisionInfoIn* info_in, const CameraInfo& cameraInfo, LineSpots& lineSpots, Intersections& intersections, std::vector<FieldFeatureInfo>& features);

    void updatePenaltyMark(const VisionInfoIn* info_in, const CameraInfo& cameraInfo, PenaltyMarkInfo& penaltyMark, std::vector<FieldFeatureInfo>& features);

    // Calculates the angle between the robot and the given field feature.
    float getRobotCentreAngle(FieldFeatureInfo feature);

    // Calculates the angle of the centre line given a line.
    float getCentreLineAngle(LineSpots::Line* line);

    // Determines the angle of a corner in the form required by localisation.
    float findCAngle(PointF& intersection, LineSpots::Line& l1, LineSpots::Line& l2);

    // Determines the angle of a T intersection in the form required by localisation.
    float findTAngle(PointF& intersection, LineSpots::Line& l);

    // Determines the gradient of a line relative to a point.
    float findGradient(LineSpots::Line& l, PointF& intersection);

	void configure(Blackboard* blackboard);

	void detect_(CameraInfo::Camera whichCamera,
                const VisionInfoIn* info_in, 
                VisionInfoMiddle* info_middle,
                VisionInfoOut* info_out);

    int OPPONENT_FIELD_BOUNDARY = static_cast<int>((FIELD_LENGTH / 2) + FIELD_LENGTH_OFFSET);
    int LEFT_FIELD_BORDER = static_cast<int>((FIELD_WIDTH / 2) + FIELD_WIDTH_OFFSET);
    int OPPONENT_PENALTY_AREA = static_cast<int>((FIELD_LENGTH / 2) - PENALTY_BOX_LENGTH);

    unsigned int lastTimestamp = 0;
    int numberOfFeaturesSent = 0;

    float bigLineThreshold = 1000.f; /**< Internal definition for a long line. TODO: Rethink this! */
    int maxTimeOffset = 30;
    float maxLineDeviationFromAssumedCenterCircle = 200.f; /**< If the distance of a short line to the center circle is larger than this, it is not considered to be on the circle. */
    float centerWeighting = 0.4f; /**< Used for computing the covariance of a line by determining the actual point for this computation, must be between 0 (-> closest point on line) and 1 (-> center of line).*/
};