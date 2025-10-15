/**
 * @file IntersectionCandidatesMiddleInfo.hpp
 *
 * This file implements a module that detects and prepares intersections candidates for classifying.
 *
 * @author Arne Böckmann
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Felix Thielke
 * @author Laurens Schiefelbein
 * @author Kevin Dehmlow
 * @author Roman Sablotny
 * @author RedbackBots
 *
 */

#pragma once

#include "perception/vision/Detector.hpp"
#include "blackboard/Blackboard.hpp"

#include "utils/ml/PatchUtilities.hpp"
#include "types/camera/CameraInfo.hpp"
#include "types/vision/ECImage.hpp"
#include "types/vision/LineSpots.hpp"
#include "types/vision/IntersectionCandidates.hpp"

#include <CompiledNN.h>

class IntersectionCandidatesMiddleInfo : public Detector {
public:

	IntersectionCandidatesMiddleInfo(Blackboard* blackboard);
  	virtual ~IntersectionCandidatesMiddleInfo();

	// Resets middle info
	virtual void resetMiddleInfo(VisionInfoMiddle* info_middle);
	virtual void resetVisionOut(VisionInfoOut* info_out);

protected:
  	virtual void detect_(const VisionInfoIn* info_in, 
                      	VisionInfoMiddle* info_middle,
                      	VisionInfoOut* info_out);
private:
    /**
     * Stores a patch from the given image coordinates as center of the patch to the given reference.
     * @param interImg image coordinates of the intersection
     * @param interField field coordinates of the intersection
     * @param[out] patch, reference where to store the patch
     * @param ecImage
     * @return True, when a patch has been correctly stored in the patch reference
     */
    bool generatePatch(const Vector2f interImg, const Vector2f& interField, Image<PixelTypes::GrayscaledPixel>& patch, ECImage& ecImage) const;

    /**
     * Returns the distance of the closer point to target.
     * @param[out] closer the point closer to the target
     * @param[out] further the point further away from the target
     */
    template<typename T>
    float getCloserPoint(const Eigen::Matrix<T, 2, 1>& a, const Eigen::Matrix<T, 2, 1>& b, const Eigen::Matrix<T, 2, 1>& target, Eigen::Matrix<T, 2, 1>& closer, Eigen::Matrix<T, 2, 1>& further) const;

    /**
     * Checks if the center of the patch is within the camera bounds.
     * @param intersectionPoint Image coordinate of the intersection
     * @param cameraInfo
     * @return True if the point is within the camera bounds. False otherwise.
     */
    bool isWithinBounds(const Vector2f& intersectionPoint, const CameraInfo& cameraInfo) const;

    /** Determines whether the point is in the line segment or not */
    bool isPointInSegment(const LineSpots::Line& line, const Vector2f& point) const;
  
    void configure(Blackboard* blackboard);

	void detect_(CameraInfo::Camera whichCamera,
                const VisionInfoIn* info_in, 
                VisionInfoMiddle* info_middle,
                VisionInfoOut* info_out);
    
    unsigned patchSize = 32; /**< size of the patch */
    float maxAllowedAngleDifference = 0.15; /**< max difference between two lines to declare as intersection*/
    float maxIntersectionGap = 400; /**< max gap between lines to declare as intersection */
    float normFactor = 4500; /**< factor to normalize the distance to the intersection from the robot */
};