/**
 * @file RobotMiddleInfo.hpp
 *
 * This file defines a module that detects SPL robots in an image using a neural network.
 *
 * @author Kelke van Lessen
 * @author Lukas Malte Monnerjahn
 * @author Fynn Boese
 * @author Bernd Poppinga
 * @author RedbackBots
 */

#pragma once

#include "perception/vision/Detector.hpp"
#include "blackboard/Blackboard.hpp"

#include "utils/defs/BallDefinitions.hpp"
#include "utils/defs/FieldDefinitions.hpp"
#include "types/camera/CameraInfo.hpp"
#include "types/field/FieldBoundary.hpp"
#include "types/vision/ECImage.hpp"
#include "types/vision/RobotObstaclesField.hpp"
#include "types/vision/RobotObstaclesImage.hpp"
#include "types/vision/RobotObstaclesIncomplete.hpp"
#include "types/LabelImage.hpp"
#include "types/math/Eigen.hpp"

#include <CompiledNN.h>
#include <CompiledNN2ONNX/CompiledNN.h>

class RobotMiddleInfo : public Detector 
{
public:
    RobotMiddleInfo(Blackboard* blackboard, asmjit::JitRuntime* jitRuntime);
    virtual ~RobotMiddleInfo();

    // Resets
    virtual void resetMiddleInfo(VisionInfoMiddle* info_middle);
    virtual void resetVisionOut(VisionInfoOut* info_out);

    /** This enumeration lists the possible classes of a image region. */
    enum Classification { // color classes for segmentation
        Horizontal,
        Vertical,
        Mixed,
        Nothing,
        Unknown,
        Default,
    };

    /** This struct represents an image region. */
    struct Region {
        Vector2i regionIndices; /**< The x- and y-indices of this region in the image. */
        std::vector<int> contrastChanges = {0, 0}; /**< Number of registered contrast changes within the region (horizontal/vertical) */
        Classification classification = Nothing; /**< The classification of this region. */
        bool clustered = false; /**< The current state of the region in terms of belonging to a cluster. */
        bool bright = false; /**< Whether the region is marked as bright. */
        int brightSpots = 0; /**< Number of bright spots in the region. */
        int maxY = -1; /**< The maximum y coordinate of all spots within the region at which a contrast change was registered. */
    };

    struct RDNetworkParameters {
        // network input shape
        unsigned int inputHeight;
        unsigned int inputWidth;
        unsigned int inputChannels;
    
        // network output shape
        unsigned int outputHeight;
        unsigned int outputWidth;
        unsigned int outputAnchors;
        unsigned int paramsPerAnchor;
    
        // class prediction features
        bool predictFallen;
    
        // indices of network outputs inside an anchor
        int confidenceIndex;
        int yMidIndex;
        int xMidIndex;
        int heightIndex;
        int widthIndex;
        int fallenClassIndex;
    
        // Allow up to 6 anchors (predictors per output pixel), not all need to be used
        std::vector<Vector2f> anchors;
    
        // network model configuration
        float sizeConversionFactor;
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
     * @param RobotObstaclesImage The representation updated.
     */
    void updateRobotObstaclesImage(RobotObstaclesImage& robotsImage);

    /**
     * This method is called when the representation provided needs to be updated.
     * @param RobotObstaclesField the representation updated.
     */
    void updateRobotObstaclesField(const VisionInfoIn* info_in, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary, ECImage& ecImage, RobotObstaclesField& robotsField, RobotObstaclesIncomplete& robotsData, RobotObstaclesIncomplete& otherObstaclesData);

    /**
     * Apply a network to extract obstacles from the image.
     * @param obstacles list of obstacle percepts to be updated
     */
    void extractImageObstaclesFromNetwork(std::vector<RobotObstaclesImage::Obstacle>& obstacles, const CameraInfo& cameraInfo, ECImage& ecImage, FieldBoundary& fieldBoundary);

    /**
     * Fill the Y-Thumbnail image with a downscaled grayscale image.
     */
    void fillGrayscaleThumbnail(ECImage& ecImage);

    /**
     * Fill the redChroma- and V-Thumbnail images with downscale images of the same size as the grayscale thumbnail.
     * First subsamples from the CameraImage to obtain the same resolution vertically and horizontally.
     * Then downscales to the expected input size.
     */
    void fillChromaThumbnails(ECImage& ecImage);

    /**
     * Applies the network on the downscaled grayscale image.
     */
    void applyGrayscaleNetwork(ECImage& ecImage);

    /**
     * Applies the network on a downscaled YUV image.
     */
    void applyColorNetwork(ECImage& ecImage);

    /**
     * This method gets the bounding boxes from the network output.
     * @param the bounding boxes
     */
    void boundingBoxes(LabelImage& labelImage, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary);

    /**
     * Computes the bounding box position and size, given a prediction form the network.
     * Computes intermediate results in place, i.e. network output becomes invalidated.
     * @param pred network prediction, also serves as output vector
     * @param y vertical position of the output cell this prediction belongs to
     * @param x horizontal position of the output cell this prediction belongs to
     * @param b index of the anchor this prediction belongs to
     */
    LabelImage::Annotation predictionToBoundingBox(Eigen::Map<Eigen::Matrix<float, 5, 1>>& pred, unsigned int y, unsigned int x, unsigned int b, const CameraInfo& cameraInfo) const;

    /**
     *
     * @param obstaclesFieldPercept representation to be updated
     * @param obstacles
     */
    void mergeObstacles(RobotObstaclesField& robotsField, RobotObstaclesIncomplete& obstaclesData, std::vector<RobotObstaclesImage::Obstacle>& obstacles, const VisionInfoIn* info_in, const CameraInfo& cameraInfo, ECImage& ecImage, RobotObstaclesIncomplete& otherObstaclesData);

    /**
     * Corrects the left and right and optionally the bottom boundary of an obstacle in the image.
     * @param trimHeight Whether the bottom boundary should be corrected.
     * @param obstacleInImage The obstacle whose boundaries should be corrected.
     */
    bool trimObstacle(bool trimHeight, RobotObstaclesImage::Obstacle& obstaclesImage, const CameraInfo& cameraInfo, ECImage& ecImage);

    /**
     * Divides the grayscale image into regions and searches them for changes in contrast.
     * @param regions The regions in which the registered contrast changes should be recorded.
     */
    void scanImage(std::vector<std::vector<Region>>& regions, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary, ECImage& ecImage);

    /**
     * Classifies image regions based on the number and type of spots contained.
     * @param regions The regions to be classified.
     */
    void classifyRegions(std::vector<std::vector<Region>>& regions, ECImage& ecImage);

        /**
     * Discards contiguous, homogeneous regions.
     * @param regions The regions to be processed.
     */
    void discardHomogeneousAreas(std::vector<std::vector<Region>>& regions);

    /**
     * Creates a previously unknown number of clusters from all image regions and stores them as obstacles.
     * @param regions The regions to be clustered.
     * @param obstacles The container to store the obstacles.
     */
    void dbScan(std::vector<std::vector<Region>>& regions, std::vector<RobotObstaclesImage::Obstacle>& obstacles, const CameraInfo& cameraInfo, ECImage& ecImage);

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
    bool expandCluster(std::vector<std::vector<RobotMiddleInfo::Region>>& regions, RobotMiddleInfo::Region& region, std::vector<Vector2i>& neighbors, std::vector<Vector2i>& cluster, Vector2i& topLeft, Vector2i& bottomRight);

    /**
     * Collects the indices of all regions with a specified classification in a specified radius around a given region.
     * @param regions The image regions to be considered.
     * @param region The region surrounding which further regions are to be looked for.
     * @param dis The maximum distance up to which further regions should be looked for.
     * @param neighbor The container to store the found region indices.
     * @param classificationToSearch The classification which regions must have in order to be collected.
     */
    void regionQuery(std::vector<std::vector<Region>>& regions, Vector2i& region, int dis, std::vector<Vector2i>& neighbors, Classification classificationToSearch = Default);

    Blackboard* blackboard = nullptr;

    Vector2i inputImageSize;

    std::unique_ptr<NeuralNetworkONNX::Model> model;
    NeuralNetworkONNX::CompiledNN network;

    Image<PixelTypes::GrayscaledPixel> grayscaleThumbnail;
    // Splitting into three images and recombining them into one is probably a performance issue
    // We need to optimize this later
    Image<PixelTypes::GrayscaledPixel> redChromaThumbnail;
    Image<PixelTypes::GrayscaledPixel> blueChromaThumbnail;
    std::vector<RobotObstaclesImage::Obstacle> obstaclesUpper;

    RDNetworkParameters networkParameters;

    [[maybe_unused]] const int HEIGHT_SHAPE_INDEX = 0;
    [[maybe_unused]] const int WIDTH_SHAPE_INDEX = 1;
    [[maybe_unused]] const int IMAGE_CHANNELS_INDEX = 2; /**< On network input */
    [[maybe_unused]] const int ANCHOR_SHAPE_INDEX = 2;   /**< On network output */
    [[maybe_unused]] const int BOX_SHAPE_INDEX = 3;

    float objectThres = 0.6f; /**< Limit from which a robot is accepted. */
    float fallenThres = 0.55f; /**< Confidence threshold for a robot to be predicted as lying on the ground */
    float nonMaximumSuppressionIoUThreshold = 0.3f; /**< Suppress non-maximal robot predictions with intersection over union above this threshold */
    unsigned int xyStep = 2; /**< Step size in x/y direction for scanning the image. */
    unsigned int xyRegions = 16; /**< Number of regions in x/y direction. */
    short minContrastDiff = 40; /**< Minimal contrast difference to consider a spot. */
    short brightnessThreshold = 100; /**< Minimal brightness to count a spot as bright. */
    unsigned char satThreshold = 40; /**< Maximum saturation to count a spot as non saturated. */
    float mixedThresh = 0.2f; /**< Ratio of the horizontal & vertical contrast changes from which a region is considered a mixed region. */
    int minHetSpots = 2; /**< Minimal number of differently classified neighbor regions to not discard a region. */
    unsigned int minNeighborPoints = 5; /**< Minimal number of neighbors to count a region as a core region (dbScan). */
    int minPixel = 20; /**< Minimal width of an obstacle in the image (in pixel). */
    float minWidthOnFieldNoMatch = 200.f; /**< Minimal width of an obstacle on the field (in mm) for which there is NO matching obstacle in the upper image. */
    bool trimObstacles = true; /**< Whether the width of obstacles should be corrected. */
    float minBeforeAfterTrimRatio = 0.32f;
    bool mergeLowerObstacles = true; /**< Whether overlapping obstacles should be merged (only for the lower camera). */
    Vector2f pRobotRotationDeviationInStand = {0.02f, 0.04f}; /**< Deviation of the rotation of the robot's torso while standing. */
    Vector2f pRobotRotationDeviation = {0.04f, 0.04f}; /**< Deviation of the rotation of the robot's torso. */
};