/**
 * @file JerseyClassifier.hpp
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

#pragma once

#include "blackboard/Blackboard.hpp"
#include "perception/vision/VisionInfoIn.hpp"
#include "utils/defs/RobotDefinitions.hpp"
#include "types/camera/CameraInfo.hpp"
#include "types/vision/ECImage.hpp"
#include "types/vision/RobotObstaclesField.hpp"
#include "types/vision/RobotObstaclesImage.hpp"
#include "types/TeamColour.hpp"
#include "types/math/Range.hpp"
#include <functional>

namespace JerseyClassifier {
    /**
     * Detect the jersey color. The method estimates the expected position of the jersey
     * in the image and samples a grid inside a parallelogram. It is checked whether more
     * samples support the own or the opponent jersey color. The features used are
     * brightness (to distinguish gray from black or white), field color, and hue.
     * If one team has the jersey color white, the results will probably misleading,
     * because arms and goalposts are also white. The team color green is not supported.
     * @param obstacleInImage The obstacle as it was detected in the image.
     * @param obstacleOnField The fields detectedJersey and ownTeam will be updated if a
     *                        jersey color was detected.
     */
    void detectJersey(const RobotObstaclesImage::Obstacle& obstacleInImage, RobotObstaclesField::Obstacle& obstacleOnField,
                        Blackboard* blackboard, const VisionInfoIn* info_in, const CameraInfo& cameraInfo, ECImage& ecImage);

    /**
     * The method determines a way to detect whether a pixel belongs to a specific jersey
     * color by comparing it to all other jersey colors used for this game.
     * @param checkColor The team color index of the jersey color that should be detected.
     * @param o1 A team color index of another color on the pitch.
     * @param o2 A team color index of another color on the pitch.
     * @param o3 A team color index of another color on the pitch.
     * @param maxBrightness The intensity of the brightest pixel below the jersey. This
     *                     functions as a reference if one of the two teams uses the
     *                     jersey color gray.
     * @return The classifier that detects whether the pixel at (x, y) (the two parameters)
     *         belongs to the jersey color of teamColor.
     */
    static std::function<bool(const int, const int)> getPixelClassifier(const TeamColour checkColor, 
                                                                    const TeamColour o1, const TeamColour o2, const TeamColour o3,
                                                                    const int maxBrightness,
                                                                    Blackboard* blackboard, ECImage& ecImage);

    static constexpr int jerseyHues[] =
    {
        255 * 7 / 8, // blue + cyan
        255 * 2 / 8, // red
        255 * 4 / 8, // yellow
        0, // black (unused)
        0, // white (unused)
        255 * 5 / 8, // green
        255 * 3 / 8, // orange
        32, // purple
        255 * 7 / 16, // brown
        0, // gray (unused)
    };

    static constexpr Rangef jerseyYRange = (300.f, 410.f); /**< The expected height range of the jersey in the image in mm. */
    static constexpr float whiteScanHeightRatio = 0.8f; /**< How high to scan for the maximum brightness between the foot point and the lower jersey edge (0..1). */
    static constexpr Rangef grayRange = (0.37f, 0.67f); /**< Which ratio range of the maximum brightness is considered to be gray? */
    static constexpr int jerseyMinYSamples = 12; /**< How many vertical samples for the jersey scan at minimum? */
    static constexpr float jerseyMaxYSamples = 20; /**< How many vertical samples for the jersey scan at maximum? */
    static constexpr float jerseyMaxXSamples = 30; /**< How many horizontal samples for the jersey scan at maximum? */
    static constexpr float relativeJerseyWidth = 0.5f; /**< The typical relative width of the jersey in the scanned region. */
    static constexpr int hueSimilarityThreshold = 24; /**< Maximum deviation from team color hue value still accepted (0 - 128). */
    static constexpr unsigned char colorDelimiter = 110; /**< Delimiter for grey, black and white against colors by saturation, only used if no color is involved. */
    static constexpr bool weighPixels = true; /**< Weigh each pixel individually or give all weight one. */
    static constexpr float minJerseyWeightRatio = 0.1f; /**< The minimum number of weighted points of a jersey color required in ratio to examined pixels . */
    static constexpr float minJerseyRatio = 0.6f; /**< The majority required of one jersey color over the other. */
    static constexpr int whiteScanOffSet = 4; /**< y-Pixel offset for additional whitescan rows*/
    static constexpr unsigned char satThreshold = 87; /**< Maximum saturation to count a spot as non saturated. */
}