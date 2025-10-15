/**
 * @file RobotLowerMiddleInfo.hpp
 *
 * This file declares a module that transplants the SegmentedObstacleImage onto the lower camera handling of the RobotMiddleInfo.
 *
 * @author Bernd Poppinga
 * @author Andreas Baude
 * @author Arne Hasselbring
 * @author RedbackBots
 */

#pragma once

#include "perception/vision/Detector.hpp"

#include "blackboard/Blackboard.hpp"
#include "utils/defs/BallDefinitions.hpp"
#include "types/camera/CameraInfo.hpp"
#include "types/field/FieldBoundary.hpp"
#include "types/vision/ECImage.hpp"
#include "types/vision/RobotObstaclesImage.hpp"
#include "types/vision/RobotObstaclesField.hpp"
#include "types/vision/RobotObstaclesIncomplete.hpp"
#include "types/vision/SegmentedObstacleImage.hpp"
#include "perception/vision/other/JerseyClassifier.hpp"
#include "types/math/Eigen.hpp"

class RobotLowerMiddleInfo : public Detector 
{
public:
    RobotLowerMiddleInfo(Blackboard* blackboard);
    virtual ~RobotLowerMiddleInfo();

    // Resets
    virtual void resetMiddleInfo(VisionInfoMiddle* info_middle);
    virtual void resetVisionOut(VisionInfoOut* info_out);

    /** This struct represents an image region. */
    struct Region
    {
        Vector2i regionIndices; // The x- and y-indices of this region in the image.
        bool isObstacle = false;
        bool clustered = false; // The current state of the region in terms of belonging to a cluster.
        int maxY = -1; // The maximum y coordinate of all spots within the region at which a contrast change was registered.
    };

protected:
    virtual void detect_(const VisionInfoIn* info_in, 
                        VisionInfoMiddle* info_middle,
                        VisionInfoOut* info_out);

private:
    void configure(Blackboard* blackboard);

    /**
     * Run detection for the given camera
     */
    void detect_(CameraInfo::Camera whichCamera,
                 const VisionInfoIn* info_in, 
                 VisionInfoMiddle* info_middle,
                 VisionInfoOut* info_out);

    /**
     * This method is called when the representation provided needs to be updated.
     * @param robotsImage The representation updated.
     */
    void updateObstaclesImage(RobotObstaclesImage& robotsImage);

    /**
     * This method is called when the representation provided needs to be updated.
     * @param robotsField The representation updated.
     */
    void updateObstaclesField(RobotObstaclesField& robotsField, RobotObstaclesIncomplete& obstaclesData, RobotObstaclesIncomplete& otherObstaclesData, SegmentedObstacleImage segmentedObstacleImage,
                                const VisionInfoIn* info_in, const CameraInfo cameraInfo, FieldBoundary& fieldBoundary, ECImage& ecImage);

     /**
     * Corrects the left and right and optionally the bottom boundary of an obstacle in the image.
     * @param trimHeight Whether the bottom boundary should be corrected.
     * @param obstacleInImage The obstacle whose boundaries should be corrected.
     */
    bool trimObstacle(bool trimHeight, RobotObstaclesImage::Obstacle& obstacleImage,
                        ECImage& ecImage);

    /**
     * Divides the grayscale image into regions and searches them for changes in contrast.
     * @param regions The regions in which the registered contrast changes should be recorded.
     */
    void scanImage(std::vector<std::vector<Region>>& regions, SegmentedObstacleImage& segmentedObstacleImage, 
                    const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary);

    /**
     * Creates a previously unknown number of clusters from all image regions and stores them as obstacles.
     * @param regions The regions to be clustered.
     * @param obstacles The container to store the obstacles.
     */
    void dbScan(std::vector<std::vector<Region>>& regions, std::vector<RobotObstaclesImage::Obstacle>& obstacles, SegmentedObstacleImage segmentedObstacleImage,
                    ECImage& ecImage);

    /**
     * Expands iteratively a cluster given a region within the cluster and its neighboring regions.
     * @param regions The regions to be clustered.
     * @param region The first region to be part of the cluster.
     * @param neighbors The neighbors of the first cluster region.
     * @param cluster The cluster to be expanded.
     * @param topLeft The indices of the top left region within a bounding box around the formed cluster.
     * @param bottomRight The indices of the bottom right region within a bounding box around the formed cluster.
     * @return Whether the cluster formed contains regions that are classified as mixed.
     */
    bool expandCluster(std::vector<std::vector<Region>>& regions, Region& region, std::vector<Vector2i>& neighbors, std::vector<Vector2i>& cluster, Vector2i& topLeft, Vector2i& bottomRight);

    /**
     * Collects the indices of all regions with a specified classification in a specified radius around a given region.
     * @param regions The image regions to be considered.
     * @param region The region surrounding which further regions are to be looked for.
     * @param dis The maximum distance up to which further regions should be looked for.
     * @param neighbor The container to store the found region indices.
     * @param classificationToSearch The classification which regions must have in order to be collected.
     */
    void regionQuery(std::vector<std::vector<Region>>& regions, Vector2i& region, int dis, std::vector<Vector2i>& neighbors);

    Blackboard* blackboard = nullptr;

    std::vector<RobotObstaclesImage::Obstacle> obstaclesUpper, obstaclesLower;

    unsigned int xyStep = 2; /**< Step size in x/y direction for scanning the image. */
    short minContrastDiff = 40; /**< Minimal contrast difference to consider a spot. */
    unsigned char satThreshold = 40; /**< Maximum saturation to count a spot as non saturated. */
    unsigned int minNeighborPoints = 5; /**< Minimal number of neighbors to count a region as a core region (dbScan). */
    int minPixel = 20; /**< Minimal width of an obstacle in the image (in pixel). */
    float minWidthOnFieldNoMatch = 200.f; /**< Minimal width of an obstacle on the field (in mm) for which there is NO matching obstacle in the upper image. */
    bool trimObstacles = false; /**< Whether the width of obstacles should be corrected. */
    float minBeforeAfterTrimRatio = 0.32f;
    bool mergeLowerObstacles = true; /**< Whether overlapping obstacles should be merged (only for the lower camera). */
    Vector2f pRobotRotationDeviationInStand = {0.02f, 0.04f}; /**< Deviation of the rotation of the robot's torso while standing. */
    Vector2f pRobotRotationDeviation = {0.04f, 0.04f};        /**< Deviation of the rotation of the robot's torso. */
};