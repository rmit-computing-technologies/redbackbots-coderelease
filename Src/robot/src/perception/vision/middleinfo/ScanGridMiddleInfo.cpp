
/**
 * @file ScanGridMiddleInfo.cpp
 *
 * Runs as a detector to provide middle information.
 * Provides the description of a grid for scanning the image. 
 * The grid resolution adapts to the camera perspective.
 * 
 * @author Thomas Röfer 
 * @author RedbackBots
 */

#include "perception/vision/middleinfo/ScanGridMiddleInfo.hpp"

#include "perception/vision/VisionInfoMiddle.hpp"
#include "utils/SpatialUtilities.hpp"
#include "types/math/Eigen.hpp"
#include "types/math/Boundary.hpp"
#include "utils/debug/Assert.hpp"
#include "utils/defs/BallDefinitions.hpp"
#include "utils/defs/FieldDefinitions.hpp"
#include "utils/Logger.hpp"

#include <algorithm>


ScanGridMiddleInfo::ScanGridMiddleInfo(Blackboard* blackboard):
    Detector("ScanGridMiddleInfo")
{
    configure(blackboard);

    // llog(INFO) << NDEBUG_LOGSYMB << "ScanGridMiddleInfo loaded" << std::endl;
}

ScanGridMiddleInfo::~ScanGridMiddleInfo() {

}

void ScanGridMiddleInfo::configure(Blackboard* blackboard) {

}

void ScanGridMiddleInfo::resetMiddleInfo(VisionInfoMiddle* info_middle) {
    info_middle->scanGrid[CameraInfo::Camera::top].clear();
    info_middle->scanGrid[CameraInfo::Camera::bot].clear();
}

void ScanGridMiddleInfo::resetVisionOut(VisionInfoOut* info_out) {
}

void ScanGridMiddleInfo::detect_(const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << __PRETTY_FUNCTION__ << std::endl;
    // llog(INFO) << "-----ScanGridMiddleInfo-----" << std::endl;

    // Top camera must run first, as the bottom camera is only computed if the top camera boundary can't be projected
    // llog(INFO) << "---Top Camera---" << std::endl;
    detect_(CameraInfo::Camera::top, info_in, info_middle, info_out);

    // TODO (TW): decide if lower field boundary is actually needed
    // llog(INFO) << "---Bot Camera---" << std::endl;
    detect_(CameraInfo::Camera::bot, info_in, info_middle, info_out);
}

void ScanGridMiddleInfo::detect_(CameraInfo::Camera whichCamera,
    const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {

    const CameraInfo& cameraInfo = info_in->cameraInfo[whichCamera];
    FieldBoundary& fieldBoundary = info_out->fieldBoundary[whichCamera];

    ScanGrid& scanGrid = info_middle->scanGrid[whichCamera];

    if(!fieldBoundary.isValid) {
        return; // Cannot compute grid without field boundary
    }

    scanGrid.fieldLimit = calcFieldLimit(info_in, cameraInfo);
    if(scanGrid.fieldLimit < 0) {
        // llog(INFO) << "- field limit below 0" << std::endl;
        return;
    }

    ImageCornersOnField lowerImageCornersOnField = calcImageCornersOnField(VerticalBoundary::LOWER, info_in, cameraInfo);
    if(!lowerImageCornersOnField.valid) {
        // llog(INFO) << "- cannot project lower image border to field" << std::endl;
        return; // Cannot project lower image border to field -> no grid
    }

    setFullResY(scanGrid, lowerImageCornersOnField, info_in, cameraInfo);
    if(!scanGrid.fullResY.empty()) {
        setLowResHorizontalLines(scanGrid, cameraInfo, fieldBoundary);
        setVerticalLines(scanGrid, lowerImageCornersOnField, info_in, cameraInfo, fieldBoundary);
    }
    else {
        // llog(INFO) << "- scangrid empty" << std::endl;
    }
}

int ScanGridMiddleInfo::calcFieldLimit(const VisionInfoIn* info_in, const CameraInfo& cameraInfo) const
{
    // llog(INFO) << "-- calcFieldLimit --" << std::endl;
    // (MO) find better way to do this
    const float opponentFieldBorder = (FIELD_LENGTH / 2) + FIELD_LENGTH_OFFSET;
    const float leftFieldBorder = (FIELD_WIDTH / 2) + FIELD_WIDTH_OFFSET;
    
    Boundaryf boundary;
    boundary.add(Vector2f(opponentFieldBorder, leftFieldBorder));
    boundary.add(Vector2f(-opponentFieldBorder, -leftFieldBorder));

    const float fieldDiagonal = Vector2f(boundary.x.getSize(), boundary.y.getSize()).norm();
    //if(!Transformation::robotWithCameraRotationToImage(Vector2f(fieldDiagonal, 0), theCameraMatrix, theCameraInfo, pointInImage))
    Vector2i pointInImage = info_in->kinematicPose.headRelRobotToImageXY(Vector2f(fieldDiagonal, 0), cameraInfo);
    // llog(INFO) << "- pointInImage.x: " << pointInImage.x();
    // llog(INFO) << " || pointInImage.y: " << pointInImage.y() << std::endl;
    if (pointInImage.x() < 0) {
        return -1; // Cannot project furthest possible point to image -> no grid in image
    }

    // If upper image border is below fieldLimit, set it to the upper border
    int fieldLimit = std::max(pointInImage.y(), 0);
    if(fieldLimit >= cameraInfo.height) {
        return -1; // Image is above field limit -> no grid in image
    }
    
    // llog(INFO) << "- fieldLimit: " << fieldLimit << std::endl;
    return fieldLimit;
}

ScanGridMiddleInfo::ImageCornersOnField ScanGridMiddleInfo::calcImageCornersOnField(VerticalBoundary boundary, const VisionInfoIn* info_in, const CameraInfo& cameraInfo) const
{
    // llog(INFO) << "-- calcImageCornersOnField --"<< std::endl;
    ImageCornersOnField imageCornersOnField;
    imageCornersOnField.valid = false;
    Vector2i leftCorner(0, boundary == VerticalBoundary::UPPER ? 0 : cameraInfo.height - 1);
    Vector2i rightCorner(cameraInfo.width, boundary == VerticalBoundary::UPPER ? 0 : cameraInfo.height - 1);
    // llog(INFO) << "- leftCorner.x: " << leftCorner.x();
    // llog(INFO) << " || leftCorner.y: " << leftCorner.y() << std::endl;
    // llog(INFO) << "- rightCorner.x: " << rightCorner.x();
    // llog(INFO) << " || rightCorner.y: " << rightCorner.y() << std::endl;

    // if(Transformation::imageToRobotWithCameraRotation(leftCorner, theCameraMatrix, theCameraInfo, imageCornersOnField.leftOnField) &&
    // Transformation::imageToRobotWithCameraRotation(rightCorner, theCameraMatrix, theCameraInfo, imageCornersOnField.rightOnField))
    imageCornersOnField.leftOnField = info_in->kinematicPose.headRelImageToRobotXY(leftCorner.cast<float>(), cameraInfo);
    // llog(INFO) << "- imageCornersOnField.leftOnField: " << imageCornersOnField.leftOnField << std::endl;
    imageCornersOnField.rightOnField = info_in->kinematicPose.headRelImageToRobotXY(rightCorner.cast<float>(), cameraInfo);
    // llog(INFO) << "- imageCornersOnField.rightOnField: " << imageCornersOnField.rightOnField << std::endl;
    if (SpatialUtilities::possiblyOnFieldRRXY(imageCornersOnField.leftOnField) && SpatialUtilities::possiblyOnFieldRRXY(imageCornersOnField.rightOnField)) {
        imageCornersOnField.valid = true;
    }

    return imageCornersOnField;
}

void ScanGridMiddleInfo::setFullResY(ScanGrid& scanGrid, ScanGridMiddleInfo::ImageCornersOnField& lowerImageCornersOnField, const VisionInfoIn* info_in, const CameraInfo& cameraInfo) const
{   
    // old scan grid calculations
    // Vector2f leftOnField = info_in->kinematicPose.headRelImageToRobotXY(Vector2f(0, cameraInfo.height - 1), cameraInfo);
    // Vector2f rightOnField = info_in->kinematicPose.headRelImageToRobotXY(Vector2f(cameraInfo.width, cameraInfo.height - 1), cameraInfo);
    // Vector2f pointOnField = (leftOnField + rightOnField) / 2.f;
    // llog(INFO) << "-- setFullResY --"<< std::endl;

    Vector2f verticalViewCenterPointOnField = (lowerImageCornersOnField.leftOnField + lowerImageCornersOnField.rightOnField) / 2.f;
    // llog(INFO) << "- initial verticalViewCenterPointOnField.x: "<< verticalViewCenterPointOnField.x() << std::endl;
    // llog(INFO) << "- initial verticalViewCenterPointOnField.y: "<< verticalViewCenterPointOnField.y() << std::endl;
    scanGrid.fullResY.reserve(cameraInfo.height);
    const float fieldStep = FIELD_LINE_WIDTH * lineWidthRatio;
    bool singleSteps = false;
    int y;
    for(y = cameraInfo.height - 1; y > scanGrid.fieldLimit;) {
        scanGrid.fullResY.emplace_back(y);
        if(singleSteps) {
            --y;
        }
        else {
            verticalViewCenterPointOnField.x() += fieldStep;
            // llog(INFO) << "--- verticalViewCenterPointOnField.x: "<< verticalViewCenterPointOnField.x() << std::endl;
            //if(!Transformation::robotWithCameraRotationToImage(verticalViewCenterPointOnField, theCameraMatrix, theCameraInfo, verticalViewCenterPointInImage))
            Vector2i verticalViewCenterPointInImage = info_in->kinematicPose.headRelRobotToImageXY(verticalViewCenterPointOnField, cameraInfo);
            // llog(INFO) << "--- verticalViewCenterPointInImage.x: "<< verticalViewCenterPointInImage.x() << std::endl;
            // llog(INFO) << "--- verticalViewCenterPointInImage.y: "<< verticalViewCenterPointInImage.y() << std::endl;
            if (verticalViewCenterPointInImage.x() < 0) {
                break;
            }
            const int y2 = y;
            y = std::min(y2 - 1, verticalViewCenterPointInImage.y());
            // llog(INFO) << "--- y: "<< y << std::endl;
            singleSteps = y2 - 1 == y;
        }
    }
    if(y < 0 && !scanGrid.fullResY.empty() && scanGrid.fullResY.back() != 0) {
        scanGrid.fullResY.emplace_back(0);
    }
}

void ScanGridMiddleInfo::setLowResHorizontalLines(ScanGrid& scanGrid, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary) const
{
    // llog(INFO) << "-- setLowResHorizontalLines --"<< std::endl;
    scanGrid.lowResHorizontalLines.reserve((cameraInfo.height / minHorizontalLowResStepSize) + 1);
    bool minSteps = false;
    size_t fullResIndex = 0;
    for(int y = scanGrid.fullResY[fullResIndex]; y > scanGrid.fieldLimit;)
    {
        // llog(INFO) << "--- y: "<< y << std::endl;
        addLowResHorizontalLine(scanGrid, y, cameraInfo, fieldBoundary);
        if(minSteps) {
            y -= minHorizontalLowResStepSize;
        }
        else {
            ++fullResIndex;
            if(fullResIndex >= scanGrid.fullResY.size()) {
                break;
            }
            const int y2 = y;
            y = std::min(y2 - minHorizontalLowResStepSize, scanGrid.fullResY[fullResIndex]);
            minSteps = y2 - minHorizontalLowResStepSize == y;
        }
    }
}

void ScanGridMiddleInfo::setVerticalLines(ScanGrid& scanGrid, ImageCornersOnField& lowerImageCornersOnField, const VisionInfoIn* info_in, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary) const
{
    // llog(INFO) << "-- setVerticalLines --"<< std::endl;
    // Determine the maximum distance between scan lines at the bottom of the image not to miss the ball.
    const int xStepUpperBound = cameraInfo.width / minNumOfLowResScanLines;
    const int maxXStep = std::min(xStepUpperBound, static_cast<int>(static_cast<float>(cameraInfo.width) * BALL_RADIUS * 2.f *
            ballWidthRatio / (lowerImageCornersOnField.leftOnField - lowerImageCornersOnField.rightOnField).norm()));
    // llog(INFO) << "- maxXStep: "<< maxXStep << std::endl;

    // Determine the maximum distance between scan lines at the top of the image not to miss the ball. Do not go below minVerticalStepSize.
    int minXStep = minVerticalStepSize;
    ImageCornersOnField upperImageCornersOnField = calcImageCornersOnField(VerticalBoundary::UPPER, info_in, cameraInfo);
    if(upperImageCornersOnField.valid) {
        minXStep = std::max(minXStep,
                static_cast<int>(static_cast<float>(cameraInfo.width) * BALL_RADIUS * 2.f * 
                ballWidthRatio / (upperImageCornersOnField.leftOnField - upperImageCornersOnField.rightOnField).norm()));
    }
    minXStep = std::min(xStepUpperBound, minXStep);
    // llog(INFO) << "- minXStep: "<< minXStep << std::endl;

    // Determine a max step size that fulfills maxXStep2 = minXStep * 2^n, maxXStep2 <= maxXStep.
    // Also compute lower y coordinates for the different lengths of scan lines.
    int maxXStep2 = minXStep;
    std::vector<int> yStarts;
    Vector2i pointInImage;
    while(maxXStep2 * 2 <= maxXStep)
    {
        float distance = getDistanceBySize(cameraInfo, BALL_RADIUS * ballWidthRatio, static_cast<float>(maxXStep2));
        // llog(INFO) << "--- distance: "<< distance << std::endl;
        // Transformation::robotWithCameraRotationToImage(Vector2f(distance, 0), theCameraMatrix, theCameraInfo, pointInImage);
        pointInImage = info_in->kinematicPose.headRelRobotToImageXY(Vector2f(distance, 0), cameraInfo);
        // llog(INFO) << "--- pointInImage.x: "<< pointInImage.x() << std::endl;
        // llog(INFO) << "--- pointInImage.y: "<< pointInImage.y() << std::endl;
        yStarts.push_back(pointInImage.y());
        maxXStep2 *= 2;
        // llog(INFO) << "--- maxXStep2: "<< maxXStep2 << std::endl;
    }
    yStarts.push_back(cameraInfo.height);

    // Determine a pattern with the different lengths of scan lines, in which the longest appears once,
    // the second longest twice, etc. The pattern starts with the longest.
    std::vector<int> yStarts2(maxXStep2 / minXStep);
    for(size_t i = 0, step = 1; i < yStarts.size(); ++i, step *= 2) {
        for(size_t j = 0; j < yStarts2.size(); j += step) {
            yStarts2[j] = yStarts[i];
        }
    }

    // Initialize the scan states and the regions.
    const int xStart = cameraInfo.width % (cameraInfo.width / minXStep - 1) / 2;
    scanGrid.verticalLines.reserve((cameraInfo.width - xStart) / minXStep);
    size_t i = yStarts2.size() / 2; // Start with the second-longest scan line.
    for(int x = xStart; x < cameraInfo.width; x += minXStep)
    {
        int yMin = std::max(scanGrid.fieldLimit, fieldBoundary.getBoundaryY(x));
        int yMax = std::min(yStarts2[i++], cameraInfo.height);
        // theBodyContour.clipBottom(x, yMax);
        yMax = std::max(1, yMax);
        yMin = std::min(yMin, yMax - 1);
        // llog(INFO) << "--- ymin: "<< yMin << std::endl;
        // llog(INFO) << "--- ymax: "<< yMax << std::endl;
        i %= yStarts2.size();
        const size_t yMaxIndexUpperBound = std::upper_bound(scanGrid.fullResY.cbegin(), scanGrid.fullResY.cend(), yMax, std::greater_equal<>())
            - scanGrid.fullResY.cbegin();
        const size_t yMaxIndex = std::min(yMaxIndexUpperBound, scanGrid.fullResY.size() - 1);
        auto greaterEqual = [](int value, const ScanGrid::ScanGridHorizontalLine& hline){return value > hline.y;};
        const size_t lowResYMaxIndexUpperBound = std::upper_bound(scanGrid.lowResHorizontalLines.cbegin(), scanGrid.lowResHorizontalLines.cend(), yMax, greaterEqual)
            - scanGrid.lowResHorizontalLines.cbegin();
        const size_t lowResYMaxIndex = scanGrid.lowResHorizontalLines.empty() ? 0 :
                                    std::min(lowResYMaxIndexUpperBound, scanGrid.lowResHorizontalLines.size() - 1);
        // llog(INFO) << "--- yMaxIndexUpperBound: "<< yMaxIndexUpperBound << std::endl;
        // llog(INFO) << "--- yMaxIndex: "<< yMaxIndex << std::endl;
        // llog(INFO) << "--- lowResYMaxIndexUpperBound: "<< lowResYMaxIndexUpperBound << std::endl;
        // llog(INFO) << "--- lowResYMaxIndex: "<< lowResYMaxIndex << std::endl;
        scanGrid.verticalLines.emplace_back(x, yMin, yMax, static_cast<unsigned>(lowResYMaxIndex), static_cast<unsigned>(yMaxIndex));
    }

    // Set low resolution scan line info
    scanGrid.lowResStep = maxXStep2 / minXStep;
    scanGrid.lowResStart = scanGrid.lowResStep / 2;
    // llog(INFO) << "- scanGrid.lowResStep: "<< scanGrid.lowResStep << std::endl;
    // llog(INFO) << "- scanGrid.lowResStart: "<< scanGrid.lowResStart << std::endl;
    
}

void ScanGridMiddleInfo::addLowResHorizontalLine(ScanGrid& scanGrid, int y, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary) const {
    int left = horizontalLeftScanStop(y, cameraInfo, fieldBoundary);
    int right = horizontalRightScanStop(y, cameraInfo, fieldBoundary);
    if(right > left) {
        scanGrid.lowResHorizontalLines.emplace_back(y, left, right);
    }
}

int ScanGridMiddleInfo::horizontalLeftScanStop(int usedY, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary) const {
    // (MO) TODO
    // most of the time everything on the left side of the body contour right-side edge is inside the body
    // const int bodyScanStart = theBodyContour.getRightEdge(usedY, theCameraInfo.width);
    // ASSERT(bodyScanStart >= 0 && bodyScanStart < theCameraInfo.width);
    int boundaryScanStart = horizontalFieldBoundaryLeftScanStop(0, usedY, cameraInfo, fieldBoundary);
    if(boundaryScanStart >= cameraInfo.width) { // may happen due to rounding
        boundaryScanStart = 0;
    }
    //return std::max(bodyScanStart, boundaryScanStart);
    return boundaryScanStart;
}

int ScanGridMiddleInfo::horizontalRightScanStop(int usedY, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary) const {
    // (MO) TODO
    // most of the time everything on the right side of the body contour left-side edge is inside the body
    // const int bodyRight = theBodyContour.getLeftEdge(usedY, theCameraInfo.width);
    // ASSERT(bodyRight > 0 && bodyRight <= theCameraInfo.width);
    int boundaryRight = horizontalFieldBoundaryRightScanStop(cameraInfo.width, usedY, fieldBoundary);
    if(boundaryRight <= 0) { // may happen due to rounding
        boundaryRight = cameraInfo.width;
    }
    // return std::min(bodyRight, boundaryRight);
    return boundaryRight;
}

int ScanGridMiddleInfo::horizontalFieldBoundaryLeftScanStop(int x, int y, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary) const
{
    int leftX = x;
    int leftY = fieldBoundary.getBoundaryY(leftX);
    if(leftY <= y) {
        return x;
    }
    for(const Vector2i& boundaryPoint : fieldBoundary.boundaryInImage) {
        if(boundaryPoint.x() < leftX) {
            continue;
        }
        if(boundaryPoint.x() >= cameraInfo.width) {
            break;
        }
        if(boundaryPoint.y() < y) {
            auto xDistance = static_cast<float>(boundaryPoint.x() - leftX);
            auto alpha = static_cast<float>(leftY - y) / static_cast<float>(leftY - boundaryPoint.y());
            return leftX + std::max(static_cast<int>(xDistance * alpha), 0);
        }
        else {
            leftX = boundaryPoint.x();
            leftY = boundaryPoint.y();
        }
    }
    int rightY = fieldBoundary.getBoundaryY(cameraInfo.width);
    if(rightY < y) {
        auto xDistance = static_cast<float>(cameraInfo.width - leftX);
        auto alpha = static_cast<float>(leftY - y) / static_cast<float>(leftY - rightY);
        return leftX + std::max(static_cast<int>(xDistance * alpha), 0);
    }
    return cameraInfo.width;
}

int ScanGridMiddleInfo::horizontalFieldBoundaryRightScanStop(int x, int y, FieldBoundary& fieldBoundary) const
{
    int rightX = x;
    int rightY = fieldBoundary.getBoundaryY(rightX);
    if(rightY <= y) {
        return x;
    }
    auto it = fieldBoundary.boundaryInImage.rbegin();
    while(it != fieldBoundary.boundaryInImage.rend())
    {
        if(it->x() >= rightX) {
            ++it;
            continue;
        }
        if(it->x() < 0) {
            break;
        }
        if(it->y() < y) {
            auto xDistance = static_cast<float>(rightX - it->x());
            auto alpha = static_cast<float>(rightY - y) / static_cast<float>(rightY - it->y());
            return rightX - std::max(static_cast<int>(xDistance * alpha), 0) + 1;
        }
        else {
            rightX = it->x();
            rightY = it->y();
        }
        ++it;
    }
    int leftY = fieldBoundary.getBoundaryY(0);
    if(leftY < y)
    {
        auto xDistance = static_cast<float>(rightX);
        auto alpha = static_cast<float>(rightY - y) / static_cast<float>(rightY - leftY);
        return rightX - std::max(static_cast<int>(xDistance * alpha), 0) + 1;
    }
    return 1;
}


float ScanGridMiddleInfo::getDistanceBySize(const CameraInfo& cameraInfo, float sizeInReality, float sizeInPixels) const {
    const float xFactor = cameraInfo.focalLength;
    return sizeInReality * xFactor / (sizeInPixels + 0.000001f);
}