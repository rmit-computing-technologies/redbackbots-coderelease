/**
 * @file IntersectionCandidatesMiddleInfo.cpp
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

#include "perception/vision/middleinfo/IntersectionCandidatesMiddleInfo.hpp"

#include "perception/vision/VisionInfoMiddle.hpp"
#include "utils/debug/Assert.hpp"
#include "utils/Logger.hpp"
#include "utils/SpatialUtilities.hpp" 
#include "types/Image.hpp"
#include "utils/math/basic_maths.hpp"

IntersectionCandidatesMiddleInfo::IntersectionCandidatesMiddleInfo(Blackboard* blackboard):
  Detector("IntersectionCandidatesMiddleInfo")
{
    configure(blackboard);

    llog(INFO) << NDEBUG_LOGSYMB << "IntersectionCandidatesMiddleInfo loaded" << std::endl;
}

IntersectionCandidatesMiddleInfo::~IntersectionCandidatesMiddleInfo() {

}

void IntersectionCandidatesMiddleInfo::configure(Blackboard* blackboard) {

}

void IntersectionCandidatesMiddleInfo::resetMiddleInfo(VisionInfoMiddle* info_middle) {
    info_middle->intersectionCandidates[CameraInfo::Camera::top].intersections.clear();
    info_middle->intersectionCandidates[CameraInfo::Camera::bot].intersections.clear();
}

void IntersectionCandidatesMiddleInfo::resetVisionOut(VisionInfoOut* info_out) {
}

void IntersectionCandidatesMiddleInfo::detect_(const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << __PRETTY_FUNCTION__ << std::endl;

    detect_(CameraInfo::Camera::top, info_in, info_middle, info_out);

    detect_(CameraInfo::Camera::bot, info_in, info_middle, info_out);
}

void IntersectionCandidatesMiddleInfo::detect_(CameraInfo::Camera whichCamera,
    const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {

    const CameraInfo& cameraInfo = info_in->cameraInfo[whichCamera];
    ECImage& ecImage = info_middle->ecImage[whichCamera];
    LineSpots& lineSpots = info_middle->lineSpots[whichCamera];

    IntersectionCandidates& intersectionCandidates = info_middle->intersectionCandidates[whichCamera];

    for(unsigned int i = 0; i < lineSpots.lines.size(); i++)
    {
        if(lineSpots.lines[i].belongsToCircle) {
            continue;
        }

        for(unsigned int j = i + 1; j < lineSpots.lines.size(); j++) {
            if(lineSpots.lines[j].belongsToCircle) {
                continue;
            }

            // get the intersection point on field
            Vector2f intersectionOnField;
            if(!Geometry::getIntersectionOfLines(lineSpots.lines[i].line, lineSpots.lines[j].line, intersectionOnField)) {
                continue;
            }

            // angle between lines in the intersection point
            const float dot = lineSpots.lines[i].line.direction.normalized().dot(lineSpots.lines[j].line.direction.normalized());
            const float angle = std::acos(crop(dot, -1.f, 1.f));
            const float angleDiff = std::abs(angle - M_PI_2);
            if(angleDiff > maxAllowedAngleDifference) {
                continue;
            }

            const LineSpots::Line& line1 = lineSpots.lines[i];
            const LineSpots::Line& line2 = lineSpots.lines[j];

            // get intersection point on Image
            // if(!Transformation::robotToImage(intersectionOnField, theCameraMatrix, theCameraInfo, intersectionOnImage) || !isWithinBounds(intersectionOnImage))
            Vector2f intersectionOnImage = info_in->kinematicPose.robotToImageXY(intersectionOnField, cameraInfo).cast<float>();
            if(!SpatialUtilities::possiblyOnFieldRRXY(intersectionOnImage) || !isWithinBounds(intersectionOnImage, cameraInfo)) {
                continue;
            }

            // check if the intersection is on the line or in threshold
            Vector2f line1CloserEnd;
            Vector2f line1FurtherEnd;
            const float line1DistFromCloserEnd = getCloserPoint(line1.firstField, line1.lastField, intersectionOnField, line1CloserEnd, line1FurtherEnd);

            Vector2f line2CloserEnd;
            Vector2f line2FurtherEnd;
            const float line2DistFromCloserEnd = getCloserPoint(line2.firstField, line2.lastField, intersectionOnField, line2CloserEnd, line2FurtherEnd);

            const bool intersectionIsOnLine1 = isPointInSegment(line1, intersectionOnField);
            const bool intersectionIsOnLine2 = isPointInSegment(line2, intersectionOnField);

            const bool line1EndInIntersection = maxIntersectionGap > line1DistFromCloserEnd;
            const bool line2EndInIntersection = maxIntersectionGap > line2DistFromCloserEnd;

            if((!intersectionIsOnLine1 && !line1EndInIntersection) || (!intersectionIsOnLine2 && !line2EndInIntersection)) {
                continue;
            }

            Intersections::Intersection::IntersectionType type;
            if(line1EndInIntersection && line2EndInIntersection)
            {
                type = Intersections::Intersection::L;
            }
            else if(!(line1EndInIntersection || line2EndInIntersection))
            {
                type = Intersections::Intersection::X;
            }
            else
            {
                if(line1EndInIntersection || line2EndInIntersection) {
                    type = Intersections::Intersection::T;
                }
                else {
                    continue;
                }
            }

            Vector2f intersectionOnFieldRelativeRobot;
            // the position of the intersection on the field needs to be recalculated again as the transformation is not linear so the maxima of the distribution (previous value of intersection) may not be at the same location as the expected value
            // theMeasurementCovariance.transformPointWithCov(intersectionOnImage, 0.f, intersectionOnFieldRelativeRobot, cov);
            intersectionOnFieldRelativeRobot = info_in->kinematicPose.imageToRobotXY(intersectionOnImage, cameraInfo);

            Image<PixelTypes::GrayscaledPixel> patch(patchSize, patchSize);
            if(!generatePatch(intersectionOnImage, intersectionOnField, patch, ecImage)) {
                continue;
            }

            int robotToIntersectionDistance = static_cast<int>(intersectionOnFieldRelativeRobot.norm()); 


            intersectionCandidates.intersections.emplace_back(type, intersectionOnField, intersectionOnImage.cast<int>(), lineSpots.lines[i].line.direction,
                                                                lineSpots.lines[j].line.direction, i, j, line1CloserEnd, line2CloserEnd,
                                                                line1FurtherEnd, line2FurtherEnd, static_cast<float>(robotToIntersectionDistance) / normFactor, patch);
        }
    }
}

bool IntersectionCandidatesMiddleInfo::generatePatch(const Vector2f interImg, const Vector2f& interField, Image<PixelTypes::GrayscaledPixel>& patch, ECImage& ecImage) const {
    const float distanceToIntersection = interField.norm();
    const unsigned int stretchingFactor = 2; // Stretch the patch along the y-Axis by this factor
    const unsigned int maxCutoutFactor = 8; // max factor of patchSize for image cutout
    // take bigger image cutout around intersection for down sampling to patchSize
    ASSERT(maxCutoutFactor >= stretchingFactor);
    const unsigned int inputResize = crop(static_cast<unsigned>(ceilf(normFactor / distanceToIntersection)), stretchingFactor, maxCutoutFactor);
    const unsigned int inputSize = patchSize * inputResize; // This is so that distant intersections do not appear too small in the patch
    const unsigned int tempOutputSize = patchSize * stretchingFactor; // Size of the patch that gets stretched
    const Vector2f center(interImg.x(), interImg.y());

    Image<PixelTypes::GrayscaledPixel> firstPatch(tempOutputSize, tempOutputSize);

    // extract patch
    PatchUtilities::extractPatch(center.cast<int>(), Vector2i(inputSize, inputSize), Vector2i(tempOutputSize, tempOutputSize), ecImage.grayscaled, firstPatch);

    // The patch height is 64p. But we need a 32x32 patch. So we cut off the first and last 16 pixels to get the center.
    const unsigned int pixelToCutOff = (tempOutputSize - patchSize) / 2;
    for(unsigned int i = pixelToCutOff; i < firstPatch.height - pixelToCutOff; i++) {
        for(unsigned int j = 0; j < firstPatch.width; j += stretchingFactor) {
            // Corrected indices for resizedPatch because we only take half of the height and every other value of the width.
            patch[i - pixelToCutOff][j - j / stretchingFactor] = firstPatch[i][j];
        }
    }
    return true;
}

template<typename T>
float IntersectionCandidatesMiddleInfo::getCloserPoint(const Eigen::Matrix<T, 2, 1>& a, const Eigen::Matrix<T, 2, 1>& b, const Eigen::Matrix<T, 2, 1>& target, Eigen::Matrix<T, 2, 1>& closer, Eigen::Matrix<T, 2, 1>& further) const
{
    auto getVector2f = [](const Eigen::Matrix<T, 2, 1>& vec) {
        return Vector2f(vec.x(), vec.y());
    };

    const float aDist = getVector2f(a - target).squaredNorm();
    const float bDist = getVector2f(b - target).squaredNorm();
    if(aDist < bDist) {
        closer = a;
        further = b;
        return std::sqrt(aDist);
    }
    closer = b;
    further = a;

    return std::sqrt(bDist);
}

bool IntersectionCandidatesMiddleInfo::isWithinBounds(const Vector2f& intersectionPoint, const CameraInfo& cameraInfo) const {
    return (intersectionPoint.x() - patchSize / 2 >= 0 &&
            intersectionPoint.x() + patchSize / 2 <= cameraInfo.width &&
            intersectionPoint.y() - patchSize / 2 >= 0 &&
            intersectionPoint.y() + patchSize / 2 <= cameraInfo.height);
}

bool IntersectionCandidatesMiddleInfo::isPointInSegment(const LineSpots::Line& line, const Vector2f& point) const
{
    const float lineSquaredNorm = (line.firstField - line.lastField).squaredNorm();
    return (line.firstField - point).squaredNorm() <= lineSquaredNorm && (line.lastField - point).squaredNorm() <= lineSquaredNorm;
}