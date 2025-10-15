/**
 * @file KeypointsDetector.hpp
 *
 * This file declares a module that subsamples a centered patch from the
 * camera image and uses MoveNet to detect keypoints in it. The keypoints
 * correspond to different body parts of a single person.
 *
 * @author Thomas Röfer
 * @author RedbackBots
*/

#pragma once

#include "perception/vision/Detector.hpp"
#include "blackboard/Blackboard.hpp"

#include "types/math/Range.hpp"
#include "utils/defs/FieldDefinitions.hpp"
//#include "Representations/Infrastructure/GameState.h"
//#include "Representations/Modeling/RobotPose.h"
#include "types/vision/RefereeKeypoints.hpp"
#include <CompiledNN2ONNX/CompiledNN.h>

class KeypointsDetector : public Detector {
public:
    KeypointsDetector(Blackboard* blackboard, asmjit::JitRuntime* jitRuntime);
    virtual ~KeypointsDetector();

    // Resets middle info
    virtual void resetMiddleInfo(VisionInfoMiddle* info_middle);
    virtual void resetVisionOut(VisionInfoOut* info_out);

protected:
    virtual void detect_(const VisionInfoIn* info_in, 
        VisionInfoMiddle* info_middle,
        VisionInfoOut* info_out);

private:
    Blackboard* blackboard = nullptr;

    int patchSize = 256; /**< The size of the patch (192, 256, 384). */
    bool patchAtTop = true; /**< Is the patch aligned with the top rather than centered? */
    float maskCenterY = 170; /**< y coordinate of a disc-shaped image area not to mask out (for observer distance of 3 m ). */
    float maskRadius = 192; /**< radius of a disc-shaped image area not to mask out (for observer distance of 3 m ). */
    int maskWidth = 120; /**< Width of a centered column in the image not to mask out (for observer distance of 3 m ). */
    float maskRecomputeThreshold = 300; /**< If the robot's position changed more than this threshold, the masks is recomputed (in mm). */
    float minConfidence = 0.1; /**< Minimum confidence required to accept a keypoint as valid. */

    NeuralNetworkONNX::CompiledNN detector;
    std::unique_ptr<NeuralNetworkONNX::Model> kpModel;
    std::vector<Rangei> mask; /**< The x ranges to keep (max exclusive) per network input row. */
    Vector2f lastRobotPosition = Vector2f::Zero(); /**< The last position of the robot when this module was used. */

    /** The format of each keypoint in the network output. */
    struct Output
    {
        float y;
        float x;
        float confidence;
    };

    void detect_(CameraInfo::Camera whichCamera,
        const VisionInfoIn* info_in, 
        VisionInfoMiddle* info_middle,
        VisionInfoOut* info_out);

    void configure(Blackboard* blackboard);

    /** Compiles the neural network. */
    void compileNetwork();

    /**
     * Create a mask which parts of the network input should be kept and which
     * should be reset.
     * @param centerX The horizontal center of the patch in the image in double-pixels (YUV422).
     * @param centerY The vertical center of the patch in the image in pixels.
     * @param height The height of the patch in the image in pixels.
     * @param width The width of the patch in the image in pixels.
     * @param patchHeight The height of the patch i.e. network input.
     * @param patchWidth The width of the patch i.e. network input.
     * @param cameraImage
     * @param blackboard
     */
    void createMask(const unsigned centerX, const unsigned centerY,
                    const int height, const int width, const int patchHeight, const int patchWidth,  
                    const CameraImage& cameraImage, Blackboard *blackboard);

    /**
     * Apply the mask by setting selected values of the network input to 0.
     * @param patchHeight The height of the patch i.e. network input.
     * @param patchWidth The width of the patch i.e. network input.
     * @param data The network input that is changed.
     */
    void applyMask(const int patchHeight, const int patchWidth, float* data) const;

    /**
     * Extract image data from a centered patch and copy it to a float array.
     * YUV pixels are converted to RGB network input.
     * @param centerX The horizontal center of the patch in the image in double-pixels (YUV422).
     * @param centerY The vertical center of the patch in the image in pixels.
     * @param height The height of the patch in pixels.
     * @param width The width of the patch in pixels.
     * @param channel The data is copied to this array.
     * @param cameraImage
     */
    void extractPatch1to1(const unsigned centerX, const unsigned centerY,
                            const int height, const int width, float* channel, const CameraImage& cameraImage) const;

    /**
     * Extract image data from a centered patch and copy it to a float array.
     * In both dimensions, 2 YUV pixels are scaled down to a single RGB network input.
     * @param centerX The horizontal center of the patch in the image in double-pixels (YUV422).
     * @param centerY The vertical center of the patch in the image in pixels.
     * @param height The height of the centered patch in pixels.
     * @param width The width of the centered patch in pixels.
     * @param channel The data is copied to this array.
     * @param cameraImage
     */
    void extractPatch2to1(const unsigned centerX, const unsigned centerY,
                            const int height, const int width, float* channel, const CameraImage& cameraImage) const;

    /**
     * Extract image data from a centered patch and copy it to a float array.
     * In both dimensions, 3 YUV pixels are scaled down to 2 RGB network inputs.
     * @param centerX The horizontal center of the patch in the image in double-pixels (YUV422).
     * @param centerY The vertical center of the patch in the image in pixels.
     * @param height The height of the centered patch in pixels.
     * @param width The width of the centered patch in pixels.
     * @param channel The data is copied to this array.
     * @param cameraImage
     */
    void extractPatch3to2(const unsigned centerX, const unsigned centerY,
                            const int height, const int width, float* channel, const CameraImage& cameraImage) const;
};