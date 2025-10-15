/**
 * @file BOPMiddleInfo.hpp
 *
 * This file declares a module that runs a neural network on a full image
 * to detect balls, obstacles and penalty marks.
 *
 * @author Arne Hasselbring
 * @author RedbackBots
 */

#pragma once

#include "perception/vision/Detector.hpp"

#include "blackboard/Blackboard.hpp"
#include "utils/defs/FieldDefinitions.hpp"
#include "types/camera/CameraImage.hpp"
#include "types/camera/CameraInfo.hpp"
#include "types/vision/PenaltyMarkRegions.hpp"
#include "types/vision/BallSpots.hpp"
#include "types/vision/SegmentedObstacleImage.hpp"
#include "types/math/Boundary.hpp"
#include "types/math/Eigen.hpp"

#include <CompiledNN.h>
#include <memory>
#include <vector>

class BOPMiddleInfo : public Detector {
public:
    BOPMiddleInfo(Blackboard* blackboard, asmjit::JitRuntime* jitRuntime);
    virtual ~BOPMiddleInfo();

    // Resets
    virtual void resetMiddleInfo(VisionInfoMiddle* info_middle);
    virtual void resetVisionOut(VisionInfoOut* info_out);

protected:
    virtual void detect_(const VisionInfoIn* info_in, 
                        VisionInfoMiddle* info_middle,
                        VisionInfoOut* info_out);

private:
    /**
     * Runs the neural network on the current camera image if it hasn't been done this frame.
     * @param cameraImage
     * @param cameraInfo
     * @return Whether there is a valid prediction for the current image.
     */
    bool apply(const CameraImage* cameraImage, const CameraInfo& cameraInfo);

    void updateBallSpots(BallSpots& ballSpots, const CameraImage* cameraImage, const CameraInfo& cameraInfo);

    void updatePenaltyMarkRegions(PenaltyMarkRegions& penaltyMarkRegions, const VisionInfoIn* info_in, const CameraImage* cameraImage, const CameraInfo& cameraInfo);

    void updateSegmentedObstacles(SegmentedObstacleImage& segmentedObstacleImage, const CameraImage* cameraImage, const CameraInfo& cameraInfo);
    
    void configure(Blackboard* blackboard);

    /**
     * Run detection for the given camera
     */
    void detect_(CameraInfo::Camera whichCamera,
                 const VisionInfoIn* info_in, 
                 VisionInfoMiddle* info_middle,
                 VisionInfoOut* info_out);

    static constexpr std::size_t ballIndex = 0; /**< Index of the ball channel. */
    static constexpr std::size_t penaltyMarkIndex = 1; /**< Index of the penalty mark channel. */
    static constexpr std::size_t obstaclesIndex = 2; /**< Index of the obstacles channel. */
    static constexpr std::size_t numOfChannels = 4; /**< Number of channels per neural network output pixel. */

    std::unique_ptr<NeuralNetwork::Model> model; /**< The model of the neural network. */
    NeuralNetwork::CompiledNN network; /**< The compiled neural network. */
    Vector2i inputSize; /**< Input size of the neural network. */
    Vector2i outputSize; /**< Output size of the neural network. */
    Vector2i scale; /**< Scale of the neural network (input size / output size). */

    unsigned lastPrediction = 0; /**< Timestamp of the last image on which the network has been run. */

    static constexpr float ballThreshold = 0.1f; /**< Threshold from which a ball spot is created. */
    static constexpr float penaltyMarkThreshold = 0.5f; /**< Threshold from which a penalty mark region is created. */
    static constexpr float obstaclesThreshold = 0.8f; /**< Threshold from which an obstacle is created. */
};