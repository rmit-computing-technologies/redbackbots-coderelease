/**
 * @file JerseyClassifier.cpp
 *
 * This file declares a module that provides functionality to classify jerseys
 * by sampling pixels in an estimated area and classifying each of them as
 * belonging to the own team or to the opponent team.
 *
* @author Thomas Röfer (the algorithm)
 * @author Lukas Malte Monnerjahn (the algorithm)
 * @author Arne Hasselbring (the module)
 * @author Tim Laue (the update for 2023)
 * @author RedbackBots
*/

#include "perception/vision/other/JerseyClassifier.hpp"

#include "types/math/Eigen.hpp"
#include <algorithm>

void JerseyClassifier::detectJersey(const RobotObstaclesImage::Obstacle& obstacleInImage, RobotObstaclesField::Obstacle& obstacleOnField,
                                        Blackboard* blackboard, const VisionInfoIn* info_in, const CameraInfo& cameraInfo, ECImage& ecImage)
{
    // obstacleInImage distance is imprecise and only used if part of the robot is in lower image
    float distance = obstacleInImage.distance * 1000;
    if(obstacleInImage.bottomFound) {
        distance = obstacleOnField.center.norm() + FOOT_LENGTH * 0.5f;
    }
    const Vector2f centerOnField = obstacleOnField.center.normalized(distance);
    // if(Transformation::robotToImage(Vector3f(centerOnField.x(), centerOnField.y(), jerseyYRange.min), theCameraMatrix, theCameraInfo, lowerInImage)
    Vector2f lowerInImage = info_in->kinematicPose.robotToImageXY(Vector2f(centerOnField.x(), centerOnField.y()), cameraInfo, static_cast<int>(JerseyClassifier::jerseyYRange.min)).cast<float>();
    // if(Transformation::robotToImage(Vector3f(centerOnField.x(), centerOnField.y(), jerseyYRange.max), theCameraMatrix, theCameraInfo, lowerInImage)
    Vector2f upperInImage = info_in->kinematicPose.robotToImageXY(Vector2f(centerOnField.x(), centerOnField.y()), cameraInfo, static_cast<int>(JerseyClassifier::jerseyYRange.max)).cast<float>();
    if(lowerInImage.x() >= 0 && upperInImage.x() >= 0)
    {
        if(lowerInImage.y() >= JerseyClassifier::jerseyMinYSamples)
        {
            const int obstacleTop = std::max(0, obstacleInImage.top);
            if(upperInImage.y() < obstacleTop)
            {
                const float interpolationFactor = (static_cast<float>(obstacleTop) - lowerInImage.y()) / (lowerInImage.y() - upperInImage.y());
                upperInImage = Vector2f(lowerInImage.x() + (lowerInImage.x() - upperInImage.x()) * interpolationFactor, obstacleTop);
            }
            if(lowerInImage.y() > static_cast<float>(cameraInfo.height - 1))
            {
                const float interpolationFactor = (static_cast<float>(cameraInfo.height - 1) - upperInImage.y()) / (lowerInImage.y() - upperInImage.y());
                lowerInImage = Vector2f(upperInImage.x() + (lowerInImage.x() - upperInImage.x()) * interpolationFactor, static_cast<float>(cameraInfo.height - 1));
            }

            const int width = obstacleInImage.right - obstacleInImage.left + 1;
            const float ySteps = std::min(lowerInImage.y() - upperInImage.y(), JerseyClassifier::jerseyMaxYSamples);
            if(ySteps < JerseyClassifier::jerseyMinYSamples)
            {
                obstacleOnField.type = RobotObstaclesField::Type::unknown;
                return;
            }
            const float yStep = (lowerInImage.y() - upperInImage.y()) / ySteps;
            const float xyStep = (lowerInImage.x() - upperInImage.x()) / ySteps;
            float left = std::max(upperInImage.x() - static_cast<float>(width) * 0.5f, 0.f);
            float right = std::min(upperInImage.x() + static_cast<float>(width) * 0.5f, static_cast<float>(cameraInfo.width) - 0.5f);
            const float xSteps = std::min((right - left), JerseyClassifier::jerseyMaxXSamples);
            const float xStep = (right - left) / xSteps;
            // this is the maximum number, could be less if the examined robot is near the image's edge
            const float examinedPixels = xSteps * ySteps;

            TeamInfo ourTeamInfo = readFrom(gameController, our_team);
            TeamInfo opponentTeamInfo = readFrom(gameController, our_team);

            // The maximum brightness is needed to determine black, white and gray relative to it.
            unsigned char maxBrightness = 0;
            if(ourTeamInfo.fieldPlayerColour == TeamColour::black || opponentTeamInfo.fieldPlayerColour == TeamColour::black ||
                ourTeamInfo.fieldPlayerColour == TeamColour::gray || opponentTeamInfo.fieldPlayerColour == TeamColour::gray ||
                ourTeamInfo.fieldPlayerColour == TeamColour::white || opponentTeamInfo.fieldPlayerColour == TeamColour::white ||
                ourTeamInfo.goalkeeperColour == TeamColour::black || opponentTeamInfo.goalkeeperColour == TeamColour::black ||
                ourTeamInfo.goalkeeperColour == TeamColour::gray || opponentTeamInfo.goalkeeperColour == TeamColour::gray ||
                ourTeamInfo.goalkeeperColour == TeamColour::white || opponentTeamInfo.goalkeeperColour == TeamColour::white)
            {
                Vector2f centerInImage = Vector2f(static_cast<float>(obstacleInImage.left + obstacleInImage.right) * 0.5f,
                        std::min(static_cast<float>(obstacleInImage.bottom), static_cast<float>(cameraInfo.height - (JerseyClassifier::whiteScanOffSet + 1))) * (1.f - JerseyClassifier::whiteScanHeightRatio)
                        + std::min(lowerInImage.y(), static_cast<float>(cameraInfo.height - (JerseyClassifier::whiteScanOffSet + 1))) * JerseyClassifier::whiteScanHeightRatio);
                float center_left = std::max(centerInImage.x() - static_cast<float>(width) * 0.5f, 0.f);
                float center_right = std::min(centerInImage.x() + static_cast<float>(width) * 0.5f, static_cast<float>(cameraInfo.width) - 0.5f);
                for(int yOffset = -1; yOffset <= 1; ++yOffset)
                for(float x = center_left; x < center_right; x += xStep)
                    maxBrightness = std::max(ecImage.grayscaled[static_cast<int>(centerInImage.y()) + JerseyClassifier::whiteScanOffSet * yOffset][static_cast<int>(x)], maxBrightness);
            }

            // Get pixel classifier
            const std::function<bool(int, int)>& isOwnFieldPlayer = getPixelClassifier(static_cast<TeamColour>(ourTeamInfo.fieldPlayerColour), static_cast<TeamColour>(opponentTeamInfo.fieldPlayerColour), static_cast<TeamColour>(opponentTeamInfo.goalkeeperColour), static_cast<TeamColour>(ourTeamInfo.goalkeeperColour), maxBrightness, blackboard, ecImage);
            const std::function<bool(int, int)>& isOpponentFieldPlayer = getPixelClassifier(static_cast<TeamColour>(opponentTeamInfo.fieldPlayerColour), static_cast<TeamColour>(ourTeamInfo.fieldPlayerColour), static_cast<TeamColour>(opponentTeamInfo.goalkeeperColour), static_cast<TeamColour>(ourTeamInfo.goalkeeperColour), maxBrightness, blackboard, ecImage);
            const std::function<bool(int, int)>& isOwnGoalkeeper = getPixelClassifier(static_cast<TeamColour>(ourTeamInfo.goalkeeperColour), static_cast<TeamColour>(ourTeamInfo.fieldPlayerColour), static_cast<TeamColour>(opponentTeamInfo.fieldPlayerColour), static_cast<TeamColour>(opponentTeamInfo.goalkeeperColour), maxBrightness, blackboard, ecImage);
            const std::function<bool(int, int)>& isOpponentGoalkeeper = getPixelClassifier(static_cast<TeamColour>(opponentTeamInfo.goalkeeperColour), static_cast<TeamColour>(opponentTeamInfo.fieldPlayerColour), static_cast<TeamColour>(ourTeamInfo.fieldPlayerColour), static_cast<TeamColour>(ourTeamInfo.goalkeeperColour), maxBrightness, blackboard, ecImage);

            float ownFieldPlayerPixels = 0;
            float opponentFieldPlayerPixels = 0;
            float ownGoalkeeperPixels = 0;
            float opponentGoalkeeperPixels = 0;

            for(float y = upperInImage.y(); y < lowerInImage.y();
                y += yStep, left = std::max(left + xyStep, 0.f), right = std::min(right + xyStep, static_cast<float>(cameraInfo.width)))
            {
                // calculations for pixel weighting
                const float ydiffFromCenter = 2 * (((lowerInImage.y() - upperInImage.y()) / 2) - (y - upperInImage.y())) / (lowerInImage.y() - upperInImage.y());
                const float yFactor = std::abs(ydiffFromCenter) < 0.5f ? 1.f : (1.f - std::abs(ydiffFromCenter)) + 0.5f;
                const float jerseyEnd = JerseyClassifier::relativeJerseyWidth - 0.1f * std::abs(ydiffFromCenter - 0.4f);
                for(float x = left; x < right; x += xStep)
                {
                    float weight = 1.f;
                    if(JerseyClassifier::weighPixels)
                    {
                        const float xdiffFromCenter = 2 * std::abs((right - left) / 2 + left - x) / (right - left);
                        const float xFactor = xdiffFromCenter < jerseyEnd ? 1 : 1 - (xdiffFromCenter - jerseyEnd);
                        weight = xFactor * yFactor;
                    }
                    if(isOwnFieldPlayer(static_cast<int>(x), static_cast<int>(y)))
                    {
                        ownFieldPlayerPixels += weight;
                    }
                    else if(isOpponentFieldPlayer(static_cast<int>(x), static_cast<int>(y)))
                    {
                        opponentFieldPlayerPixels += weight;
                    }
                    else if(isOpponentGoalkeeper(static_cast<int>(x), static_cast<int>(y)))
                    {
                        opponentGoalkeeperPixels += weight;
                    }
                    else if(isOwnGoalkeeper(static_cast<int>(x), static_cast<int>(y)))
                    {
                        ownGoalkeeperPixels += weight;
                    }
                }
                // threshold to counter white robot parts or green field background being classified as opponent jersey
                if((static_cast<TeamColour>(ourTeamInfo.fieldPlayerColour) == TeamColour::white || static_cast<TeamColour>(opponentTeamInfo.fieldPlayerColour) == TeamColour::green))
                {
                    const float threshold = 1.5f * (ownFieldPlayerPixels + ownGoalkeeperPixels) + (examinedPixels) / 5;
                    opponentFieldPlayerPixels = opponentFieldPlayerPixels <= threshold ? 0 : opponentFieldPlayerPixels - threshold;
                }
                if((static_cast<TeamColour>(ourTeamInfo.goalkeeperColour) == TeamColour::white || static_cast<TeamColour>(opponentTeamInfo.goalkeeperColour) == TeamColour::green))
                {
                    const float threshold = 1.5f * (ownFieldPlayerPixels + ownGoalkeeperPixels) + (examinedPixels) / 5;
                    opponentGoalkeeperPixels = opponentGoalkeeperPixels <= threshold ? 0 : opponentGoalkeeperPixels - threshold;
                }
                const float minJerseyWeight = examinedPixels * JerseyClassifier::minJerseyWeightRatio;
                const float overallPixels = ownFieldPlayerPixels + ownGoalkeeperPixels + opponentGoalkeeperPixels + opponentFieldPlayerPixels;
                if((ownFieldPlayerPixels > minJerseyWeight) && (ownFieldPlayerPixels > overallPixels * JerseyClassifier::minJerseyRatio))
                {
                    obstacleOnField.type = RobotObstaclesField::ownPlayer;
                }
                else if((opponentFieldPlayerPixels > minJerseyWeight) && (opponentFieldPlayerPixels > overallPixels * JerseyClassifier::minJerseyRatio))
                {
                    obstacleOnField.type = RobotObstaclesField::opponentPlayer;
                }
                else if((ownGoalkeeperPixels > minJerseyWeight) && (ownGoalkeeperPixels > overallPixels * JerseyClassifier::minJerseyRatio))
                {
                    obstacleOnField.type = RobotObstaclesField::ownGoalkeeper;
                }
                else if((opponentGoalkeeperPixels > minJerseyWeight) && (opponentGoalkeeperPixels > overallPixels * JerseyClassifier::minJerseyRatio))
                {
                    obstacleOnField.type = RobotObstaclesField::opponentGoalkeeper;
                }
                else
                {
                    obstacleOnField.type = RobotObstaclesField::unknown;
                }
            }
        }
    }
}

std::function<bool(int, int)> JerseyClassifier::getPixelClassifier(const TeamColour checkColor, 
                                                                        const TeamColour o1, const TeamColour o2, const TeamColour o3,
                                                                        const int maxBrightness,
                                                                        Blackboard* blackboard, ECImage& ecImage)
{
    const int checkHue = JerseyClassifier::jerseyHues[checkColor];
    const int o1Hue = JerseyClassifier::jerseyHues[o1];
    const int o2Hue = JerseyClassifier::jerseyHues[o2];
    const int o3Hue = JerseyClassifier::jerseyHues[o3];
    const Rangei grayRange = Rangei(static_cast<int>(maxBrightness * grayRange.min),
                                    static_cast<int>(maxBrightness * grayRange.max));

    bool checkIsBlack = checkColor == TeamColour::black;
    bool othersAreAllColorful = o1 != TeamColour::black && o1 != TeamColour::gray && o1 != TeamColour::white &&
                                o2 != TeamColour::black && o2 != TeamColour::gray && o2 != TeamColour::white &&
                                o3 != TeamColour::black && o3 != TeamColour::gray && o3 != TeamColour::white;

    // Black jersey is compared to colorful options:
    if(checkIsBlack && othersAreAllColorful) {
        return [grayRange, o1Hue, o2Hue, o3Hue, ecImage](const int x, const int y)
        {
            return ecImage.saturated[y][x] < JerseyClassifier::satThreshold && ecImage.grayscaled[y][x] <= grayRange.min &&
                    std::abs(static_cast<signed char>(ecImage.hued[y][x] - o1Hue)) > JerseyClassifier::hueSimilarityThreshold &&
                    std::abs(static_cast<signed char>(ecImage.hued[y][x] - o2Hue)) > JerseyClassifier::hueSimilarityThreshold &&
                    std::abs(static_cast<signed char>(ecImage.hued[y][x] - o3Hue)) > JerseyClassifier::hueSimilarityThreshold;
        };
    }
    // All jerseys are colorful, no black/gray/white involved at all:
    if(!checkIsBlack && othersAreAllColorful) {
        return [checkHue, o1Hue, o2Hue, o3Hue, ecImage](const int x, const int y)
        {
            const int hue = ecImage.hued[y][x];
            return std::abs(static_cast<signed char>(hue - checkHue)) < std::min(std::abs(static_cast<signed char>(hue - o1Hue)), JerseyClassifier::hueSimilarityThreshold) &&
                    std::abs(static_cast<signed char>(hue - checkHue)) < std::min(std::abs(static_cast<signed char>(hue - o2Hue)), JerseyClassifier::hueSimilarityThreshold) &&
                    std::abs(static_cast<signed char>(hue - checkHue)) < std::min(std::abs(static_cast<signed char>(hue - o3Hue)), JerseyClassifier::hueSimilarityThreshold);
        };
    }
    // We check the black jersey against a set of others that contain at least once white or gray
    if(checkIsBlack && !othersAreAllColorful) {
        return [o1Hue, o2Hue, o3Hue, o1, o2, o3, grayRange, ecImage](const int x, const int y)
        {
            bool checkO1 = false;
            if(o1 == TeamColour::gray || o1 == TeamColour::white) {
                checkO1 = ecImage.saturated[y][x] < JerseyClassifier::colorDelimiter && ecImage.grayscaled[y][x] < grayRange.min;
            }
            else {
                checkO1 = ecImage.saturated[y][x] < JerseyClassifier::satThreshold && ecImage.grayscaled[y][x] <= grayRange.min &&
                            std::abs(static_cast<signed char>(ecImage.hued[y][x] - o1Hue)) > JerseyClassifier::hueSimilarityThreshold;
            }
            if(!checkO1) {
                return false;
            }

            bool checkO2 = false;
            if(o2 == TeamColour::gray || o2 == TeamColour::white) {
                checkO2 = ecImage.saturated[y][x] < JerseyClassifier::colorDelimiter && ecImage.grayscaled[y][x] < grayRange.min;
            }
            else {
                checkO2 = ecImage.saturated[y][x] < JerseyClassifier::satThreshold && ecImage.grayscaled[y][x] <= grayRange.min &&
                            std::abs(static_cast<signed char>(ecImage.hued[y][x] - o2Hue)) > JerseyClassifier::hueSimilarityThreshold;
            }
            if(!checkO2) {
                return false;
            }

            bool checkO3 = false;
            if(o3 == TeamColour::gray || o3 == TeamColour::white) {
                checkO3 = ecImage.saturated[y][x] < JerseyClassifier::colorDelimiter && ecImage.grayscaled[y][x] < grayRange.min;
            }
            else {
                checkO3 = ecImage.saturated[y][x] < JerseyClassifier::satThreshold && ecImage.grayscaled[y][x] <= grayRange.min &&
                            std::abs(static_cast<signed char>(ecImage.hued[y][x] - o3Hue)) > JerseyClassifier::hueSimilarityThreshold;
            }

            return checkO3;
        };
    }
    // We check a colored jersey against a set of others that contain at least once white or gray or black
    else {
        return [checkHue, grayRange, o1Hue, o2Hue, o3Hue, o1, o2, o3, ecImage](const int x, const int y)
        {
            const int hue = ecImage.hued[y][x];
            bool checkO1 = false;
            if(o1 == TeamColour::black || o1 == TeamColour::gray || o1 == TeamColour::white) {
                checkO1 = std::abs(static_cast<signed char>(ecImage.hued[y][x] - checkHue)) <= JerseyClassifier::hueSimilarityThreshold;
            }
            else {
                checkO1 = std::abs(static_cast<signed char>(hue - checkHue)) < std::min(std::abs(static_cast<signed char>(hue - o1Hue)), JerseyClassifier::hueSimilarityThreshold);
            }
            if(!checkO1) {
                return false;
            }

            bool checkO2 = false;
            if(o2 == TeamColour::black || o2 == TeamColour::gray || o2 == TeamColour::white) {
                checkO2 = std::abs(static_cast<signed char>(ecImage.hued[y][x] - checkHue)) <= JerseyClassifier::hueSimilarityThreshold;
            }
            else {
                checkO2 = std::abs(static_cast<signed char>(hue - checkHue)) < std::min(std::abs(static_cast<signed char>(hue - o2Hue)), JerseyClassifier::hueSimilarityThreshold);
            }
            if(!checkO2) {
                return false;
            }

            bool checkO3 = false;
            if(o3 == TeamColour::black || o3 == TeamColour::gray || o3 == TeamColour::white) {
                checkO3 = std::abs(static_cast<signed char>(ecImage.hued[y][x] - checkHue)) <= JerseyClassifier::hueSimilarityThreshold;
            }
            else {
                checkO3 = std::abs(static_cast<signed char>(hue - checkHue)) < std::min(std::abs(static_cast<signed char>(hue - o3Hue)), JerseyClassifier::hueSimilarityThreshold);
            }
            return checkO3;
        };
    }
}
