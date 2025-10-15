/**
 * @file RobotLowerMiddleInfo.cpp
 *
 * This file implements a module that transplants the SegmentedObstacleImage onto the lower camera handling of the RobotMiddleInfo.
 *
 * @author Bernd Poppinga
 * @author Andreas Baude
 * @author Arne Hasselbring
 * @author RedbackBots
 */

#include "perception/vision/middleinfo/RobotLowerMiddleInfo.hpp"

#include "perception/vision/VisionInfoMiddle.hpp"
#include "perception/vision/VisionInfoOut.hpp"
#include "perception/vision/other/JerseyClassifier.hpp"
#include "utils/SpatialUtilities.hpp"
#include "utils/math/basic_maths.hpp"
#include "utils/debug/Assert.hpp"

RobotLowerMiddleInfo::RobotLowerMiddleInfo(Blackboard* blackboard):
    Detector("RobotLowerMiddleInfo")
{
    configure(blackboard);

    llog(INFO) << NDEBUG_LOGSYMB << "Robot Lower Detector Models loaded and compiled" << std::endl;
}

RobotLowerMiddleInfo::~RobotLowerMiddleInfo() {
    blackboard = nullptr;
}

void RobotLowerMiddleInfo::configure(Blackboard* blackboard) {
    this->blackboard = blackboard;
}

void RobotLowerMiddleInfo::resetMiddleInfo(VisionInfoMiddle* info_middle) {
    info_middle->robotsImage[CameraInfo::Camera::bot].obstacles.clear();

    info_middle->obstaclesData[CameraInfo::Camera::bot].scanScores.clear();
}

void RobotLowerMiddleInfo::resetVisionOut(VisionInfoOut* info_out) {
}

void RobotLowerMiddleInfo::detect_(const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << __PRETTY_FUNCTION__ << std::endl;

    // Top camera - done by RobotMiddleInfo
    // detect_(CameraInfo::Camera::top, info_in, info_middle, info_out);

    // Bottom camera - done by RobotLowerMiddleInfo
    detect_(CameraInfo::Camera::bot, info_in, info_middle, info_out);
}

void RobotLowerMiddleInfo::detect_(CameraInfo::Camera whichCamera,
    const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {

    const CameraInfo& cameraInfo = info_in->cameraInfo[whichCamera];
    FieldBoundary& fieldBoundary = info_out->fieldBoundary[whichCamera];
    ECImage& ecImage = info_middle->ecImage[whichCamera];
    RobotObstaclesField& robotsField = info_middle->robotsField[whichCamera];
    RobotObstaclesImage& robotsImage = info_middle->robotsImage[whichCamera];
    RobotObstaclesIncomplete& obstaclesData = info_middle->obstaclesData[whichCamera];
    RobotObstaclesIncomplete& otherObstaclesData = info_middle->obstaclesData[whichCamera];
    SegmentedObstacleImage& segmentedObstacleImage = info_middle->segmentedObstacleImage;
    
    updateObstaclesField(robotsField, obstaclesData, otherObstaclesData, segmentedObstacleImage, info_in, cameraInfo, fieldBoundary, ecImage);
    updateObstaclesImage(robotsImage);
}

void RobotLowerMiddleInfo::updateObstaclesImage(RobotObstaclesImage& robotsImage)
{
    robotsImage.obstacles = obstaclesLower;
}

void RobotLowerMiddleInfo::updateObstaclesField(RobotObstaclesField& robotsField, RobotObstaclesIncomplete& obstaclesData, RobotObstaclesIncomplete& otherObstaclesData, SegmentedObstacleImage segmentedObstacleImage,
                                                const VisionInfoIn* info_in, const CameraInfo cameraInfo, FieldBoundary& fieldBoundary, ECImage& ecImage)
{
    std::vector<RobotObstaclesImage::Obstacle>& obstacles = obstaclesLower;
    obstacles.clear();
    robotsField.obstacles.clear();
    obstaclesData.incompleteObstacles.clear();

    if(!fieldBoundary.isValid || ecImage.grayscaled.width * ecImage.grayscaled.height == 0 || segmentedObstacleImage.obstacle.width * segmentedObstacleImage.obstacle.height == 0) {
        return;
    }
    ASSERT(xyStep < ecImage.grayscaled.width && xyStep > 0 && xyStep < ecImage.grayscaled.height && xyStep > 0);

    std::vector<std::vector<Region>> regions(segmentedObstacleImage.obstacle.height, std::vector<Region>(segmentedObstacleImage.obstacle.width, Region()));
    scanImage(regions, segmentedObstacleImage, cameraInfo, fieldBoundary);
    dbScan(regions, obstacles, segmentedObstacleImage, ecImage);

    bool mergeObstacles = false;
    do
    {
        mergeObstacles = mergeLowerObstacles && !mergeObstacles && obstacles.size() >= 2;
        auto it = obstacles.begin();
        while(it != obstacles.end())
        {
            bool validObstacle;
            RobotObstaclesImage::Obstacle& obstacleInImage = *it;
            RobotObstaclesField::Obstacle obstacleOnField;

            Matrix2f leftCovariance, rightCovariance;

            obstacleOnField.left = info_in->kinematicPose.imageToRobotXY(Vector2i(obstacleInImage.left, obstacleInImage.bottom).cast<float>(), cameraInfo);
            obstacleOnField.right = info_in->kinematicPose.imageToRobotXY(Vector2i(obstacleInImage.right, obstacleInImage.bottom).cast<float>(), cameraInfo);

            if((validObstacle = (
                SpatialUtilities::possiblyOnFieldRRXY(obstacleOnField.left) && SpatialUtilities::possiblyOnFieldRRXY(obstacleOnField.right))))
            {
                obstacleOnField.fallen = obstacleInImage.fallen;
                obstacleOnField.type = RobotObstaclesField::unknown;
                obstacleOnField.center = (obstacleOnField.left + obstacleOnField.right) * 0.5f;

                const Rangea range(obstacleOnField.right.angle(), obstacleOnField.left.angle());
                if((validObstacle = obstacleInImage.top < static_cast<int>(ecImage.grayscaled.height / segmentedObstacleImage.obstacle.height) ||
                                    static_cast<float>(obstacleInImage.bottom - obstacleInImage.top) / ecImage.grayscaled.height > 0.5f))
                {
                    if(!mergeObstacles) {
                        JerseyClassifier::detectJersey(obstacleInImage, obstacleOnField, blackboard, info_in, cameraInfo, ecImage);
                    }
                    float minWidth = minWidthOnFieldNoMatch;
                    bool hasMatch = false;
                    for(const RobotObstaclesField::Obstacle& incompleteObstacle : otherObstaclesData.incompleteObstacles)
                    {
                        // const Pose2f inverseOdometryOffset = theOdometryData.inverse() * theOtherOdometryData;
                        const Rangea rangeUpper((incompleteObstacle.right).angle(), (incompleteObstacle.left).angle());
                        if((hasMatch = (range.min <= rangeUpper.max && rangeUpper.min <= range.max)))
                        {
                            obstacleOnField.type = incompleteObstacle.type;
                            minWidth = minWidthOnFieldNoMatch / 2;
                            break;
                        }
                    }
                    obstacleOnField.fallen = !hasMatch && (obstacleInImage.right - obstacleInImage.left >= static_cast<int>(ecImage.grayscaled.width / 2) ||
                                                            obstacleInImage.bottom - obstacleInImage.top >= static_cast<int>(ecImage.grayscaled.height / 2));

                    if((validObstacle = (obstacleOnField.right - obstacleOnField.left).squaredNorm() >= (minWidth*minWidth)) && !mergeObstacles)
                    {
                        const_cast<RobotObstaclesIncomplete&>(obstaclesData).incompleteObstacles.emplace_back(obstacleOnField);
                        robotsField.obstacles.emplace_back(obstacleOnField);
                    }
                    else if(!validObstacle && (obstacleOnField.right - obstacleOnField.left).squaredNorm() >= SQUARE(minWidthOnFieldNoMatch / 2)) {
                        const_cast<RobotObstaclesIncomplete&>(obstaclesData).incompleteObstacles.emplace_back(obstacleOnField);
                    }
                }
            }
            it = validObstacle ? it + 1 : obstacles.erase(it);
        }

        if(mergeObstacles)
        {
            bool mergedSomething;
            do
            {
                std::vector<bool> merged(obstacles.size(), false);
                std::vector<RobotObstaclesImage::Obstacle> mergedObstacles;
                mergedSomething = false;
                for(size_t i = 0; i < obstacles.size(); ++i)
                {
                if(merged[i]) continue;
                RobotObstaclesImage::Obstacle& a = obstacles[i];
                const Rangei obstacleRange = Rangei(a.left, a.right);

                for(size_t j = i + 1; j < obstacles.size(); ++j)
                {
                    RobotObstaclesImage::Obstacle& b = obstacles[j];
                    const Rangei otherObstacleRange = Rangei(b.left, b.right);
                    if(std::min(obstacleRange.max, otherObstacleRange.max) - std::max(obstacleRange.min, otherObstacleRange.min) > 0)
                    {
                        a.top = std::min(a.top, b.top);
                        a.bottom = std::max(a.bottom, b.bottom);
                        a.left = std::min(a.left, b.left);
                        a.right = std::max(a.right, b.right);
                        mergedObstacles.emplace_back(a);
                        merged[i] = true;
                        merged[j] = true;
                        mergedSomething = true;
                    }
                }
                if(!merged[i]) mergedObstacles.emplace_back(a);
                }
                obstacles = mergedObstacles;
            }
            while(mergedSomething);
        }
    }
    while(mergeObstacles);
}

bool RobotLowerMiddleInfo::trimObstacle(bool trimHeight, RobotObstaclesImage::Obstacle& obstacleImage,
                                        ECImage& ecImage)
{
    const int minX = std::max(obstacleImage.left, 0);
    const int minY = std::max(obstacleImage.top, 0);
    const int maxX = std::min(obstacleImage.right, static_cast<int>(ecImage.grayscaled.width)), maxY = std::min(obstacleImage.bottom, static_cast<int>(ecImage.grayscaled.height));

    int stepSize = xyStep;
    std::vector<std::vector<int>> limits = {{}, {}};
    for(int y = minY; y < maxY; y += stepSize)
    {
        int offset = minX, step = xyStep;
        for(int side = 0; side < 2; ++side, offset = maxX - 1, step *= -1)
        {
            if((side == 0 && minX <= 10) || (side == 1 && maxX >= static_cast<int>(ecImage.grayscaled.width - 10))) {
                continue;
            }

            const PixelTypes::GrayscaledPixel* secondSpot = ecImage.grayscaled[y] + offset;
            short firstSpot = *secondSpot;;
            std::function<bool(int)> comp = (side == 0) ? static_cast<std::function<bool(int)>>([maxX, stepSize](int a) -> bool {return a < maxX - stepSize;})
                                                        : static_cast<std::function<bool(int)>>([minX, stepSize](int a) -> bool {return a >= minX + stepSize;});
            for(int x = offset; comp(x); x += step, firstSpot = *secondSpot)
            {
                secondSpot += step;
                if(std::abs(*secondSpot - firstSpot) > minContrastDiff / 2)
                {
                    limits[side].emplace_back(x - step / 2);
                    break;
                }
            }
        }
    }
    std::sort(limits[0].begin(), limits[0].end());
    if(!limits[0].empty()) obstacleImage.left = limits[0][limits[0].size() / 2];
    std::sort(limits[1].begin(), limits[1].end());
    if(!limits[1].empty()) obstacleImage.right = limits[1][limits[1].size() / 2];

    if(trimHeight)
    {
        const Rangei yRange = Rangei(minY + (maxY - minY) / 4, maxY - 1);
        int step = stepSize, upperY = -1, lowerY = -1, botCan = obstacleImage.bottom;
        for(int side = 0; side < 2; ++side, step *= -1)
        {
        bool satRow = false, nonSatRow = false;
        for(int y = (minY + maxY) / 2; yRange.isInside(y) && (!nonSatRow || !satRow); y += step)
        {
            const PixelTypes::GrayscaledPixel* spot = ecImage.saturated[y] + obstacleImage.left;
            int x = obstacleImage.left;
            for(; x < obstacleImage.right && *spot > satThreshold; x += stepSize, spot += stepSize);

            if(x >= obstacleImage.right) {
                satRow = true;
            }
            else
            {
                botCan = y;
                nonSatRow = true;
            }
            if(satRow && nonSatRow)(side == 0 ? lowerY : upperY) = botCan;
        }
        }
        obstacleImage.bottom = upperY != -1 ? upperY : (lowerY != -1 ? lowerY : obstacleImage.bottom);
    }
    //Vector2f relativePosition;
    //Geometry::Circle ball;
    //const float radius = Transformation::imageToRobotHorizontalPlane(Vector2f((obstacleInImage.right + obstacleInImage.left) / 2, (obstacleInImage.top + obstacleInImage.bottom) / 2), theBallSpecification.radius, theCameraMatrix, theCameraInfo, relativePosition)
    //                    && Projection::calculateBallInImage(relativePosition, theCameraMatrix, theCameraInfo, theBallSpecification.radius, ball) ? ball.radius : -1.f;
    const int minWidthInImage = minPixel; // std::max(minPixel, static_cast<int>(radius));
    return obstacleImage.right - obstacleImage.left >= minWidthInImage && obstacleImage.bottom - obstacleImage.top >= minWidthInImage &&
            static_cast<float>(obstacleImage.right - obstacleImage.left) / static_cast<float>(maxX - minX) >= minBeforeAfterTrimRatio;
}

void RobotLowerMiddleInfo::scanImage(std::vector<std::vector<Region>>& regions, SegmentedObstacleImage& segmentedObstacleImage, 
                                    const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary)
{
    const unsigned int xScale = cameraInfo.width / segmentedObstacleImage.obstacle.width;
    const unsigned int yScale = cameraInfo.height / segmentedObstacleImage.obstacle.height;

    std::vector<std::pair<int, int>> yLimits(segmentedObstacleImage.obstacle.width);
    for(unsigned int x = xScale / 2, index = 0; index < segmentedObstacleImage.obstacle.width; x += xScale, ++index)
        // yLimits[index] = std::make_pair(fieldBoundary.getBoundaryY(x), theBodyContour.getBottom(x, cameraInfo.height));
        yLimits[index] = std::make_pair(fieldBoundary.getBoundaryY(x), cameraInfo.height);

    for(unsigned int y = 0; y < segmentedObstacleImage.obstacle.height; ++y)
    {
        const int yFullResolution = y * yScale;
        for(unsigned int x = 0; x < segmentedObstacleImage.obstacle.width; ++x)
        {
            const auto& [yMin, yMax] = yLimits[x];
            regions[y][x].regionIndices = Vector2i(x, y);
            if(yFullResolution > yMin && yFullResolution < yMax && segmentedObstacleImage.obstacle[y][x] > 128)
            {
                regions[y][x].isObstacle = true;
                regions[y][x].maxY = yFullResolution;
            }
        }
    }
}

void RobotLowerMiddleInfo::dbScan(std::vector<std::vector<Region>>& regions, std::vector<RobotObstaclesImage::Obstacle>& obstacles, SegmentedObstacleImage segmentedObstacleImage,
                                ECImage& ecImage)
{
    for(std::vector<Region>& horizontalRegionLine : regions) {
        for(Region& region : horizontalRegionLine)
        {
            if(!region.isObstacle || region.clustered) {
                continue;
            }
            std::vector<Vector2i> neighbors;
            regionQuery(regions, region.regionIndices, 1, neighbors);
            if(neighbors.size() >= minNeighborPoints)
            {
                std::vector<Vector2i> cluster;
                Vector2i topLeft = region.regionIndices, bottomRight = Vector2i(region.regionIndices.x(), region.maxY);
                if(expandCluster(regions, region, neighbors, cluster, topLeft, bottomRight))
                {
                    RobotObstaclesImage::Obstacle obstacle;
                    obstacle.top = topLeft.y() * (ecImage.grayscaled.height / segmentedObstacleImage.obstacle.height);
                    obstacle.bottom = bottomRight.y();
                    obstacle.left = topLeft.x() * (ecImage.grayscaled.width / segmentedObstacleImage.obstacle.width);
                    obstacle.right = (bottomRight.x() + 1) * (ecImage.grayscaled.width / segmentedObstacleImage.obstacle.width);
                    obstacle.bottomFound = true;
                    obstacle.fallen = false;

                    if(!trimObstacles || trimObstacle(true, obstacle, ecImage)) {
                        obstacles.emplace_back(obstacle);
                    }
                }
            }
        }
    }
}

bool RobotLowerMiddleInfo::expandCluster(std::vector<std::vector<Region>>& regions, Region& region, std::vector<Vector2i>& neighbors, std::vector<Vector2i>& cluster, Vector2i& topLeft, Vector2i& bottomRight)
{
    cluster.emplace_back(region.regionIndices);
    region.clustered = true;

    for(unsigned int i = 0; i < neighbors.size(); ++i)
    {
        Vector2i& neighborIndices = neighbors[i];
        Region& currentRegion = regions[neighborIndices.y()][neighborIndices.x()];
        if(currentRegion.clustered)
        continue;

        cluster.emplace_back(neighborIndices);
        currentRegion.clustered = true;
        if(currentRegion.isObstacle)
        {
            topLeft = Vector2i(std::min(currentRegion.regionIndices.x(), topLeft.x()), std::min(currentRegion.regionIndices.y(), topLeft.y()));
            bottomRight = Vector2i(std::max(currentRegion.regionIndices.x(), bottomRight.x()), std::max(currentRegion.maxY, bottomRight.y()));
        }

        std::vector<Vector2i> nextNeighbors;
        regionQuery(regions, neighborIndices, 1, nextNeighbors);
        if(nextNeighbors.size() >= minNeighborPoints)
        for(Vector2i& nextNeighbor : nextNeighbors) {
            if(!regions[nextNeighbor.y()][nextNeighbor.x()].clustered) {
                neighbors.emplace_back(nextNeighbor);
            }
        }
    }
    for(int yRegionIndex = topLeft.y() - 1; yRegionIndex >= 0 && yRegionIndex == topLeft.y() - 1; --yRegionIndex) {
        for(int xRegionIndex = topLeft.x(); xRegionIndex <= bottomRight.x(); ++xRegionIndex) {
            if(regions[yRegionIndex][xRegionIndex].isObstacle)
            {
                cluster.emplace_back(Vector2i(xRegionIndex, yRegionIndex));
                topLeft = Vector2i(topLeft.x(), yRegionIndex);
            }
        }
    }
    return true;
}

void RobotLowerMiddleInfo::regionQuery(std::vector<std::vector<Region>>& regions, Vector2i& regionIndices, int dis, std::vector<Vector2i>& neighbors)
{
    for(int y = std::max(regionIndices.y() - dis, 0); y < std::min(regionIndices.y() + dis + 1, static_cast<int>(regions.size())); ++y) {
        for(int x = std::max(regionIndices.x() - dis, 0); x < std::min(regionIndices.x() + dis + 1, static_cast<int>(regions[y].size())); ++x) {
            if(regions[y][x].isObstacle) {
                neighbors.emplace_back(Vector2i(x, y));
            }
        }
    }
}