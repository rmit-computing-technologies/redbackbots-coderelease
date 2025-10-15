/**
 * @file BallDetector.hpp
 *
 * This file declares a module that detects balls in images with a neural network.
 *
 * @author Bernd Poppinga
 * @author Felix Thielke
 * @author Gerrit Felsch
 * @author RedBackBots
 */

#pragma once


#include "perception/vision/Detector.hpp"
#include "blackboard/Blackboard.hpp"

#include "utils/defs/BallDefinitions.hpp"
#include "utils/defs/FieldDefinitions.hpp"
#include "types/camera/CameraInfo.hpp"
#include "types/vision/BallSpots.hpp"
#include "types/vision/PreviousBalls.hpp"
#include "types/vision/ECImage.hpp"
#include "types/BallInfo.hpp"
#include "utils/ml/PatchUtilities.hpp"


#include "types/math/Eigen.hpp"
#include <CompiledNN.h>

class BallDetector : public Detector
{
public:
    enum NormalizationMode {
        none,
        normalizeContrast,
        normalizeBrightness,
    };

    BallDetector(Blackboard* blackboard, asmjit::JitRuntime* jitRuntime);
    virtual ~BallDetector();

    // Resets middle info
    virtual void resetMiddleInfo(VisionInfoMiddle* info_middle);
    virtual void resetVisionOut(VisionInfoOut* info_out);

protected:
    virtual void detect_(const VisionInfoIn* info_in, 
                      VisionInfoMiddle* info_middle,
                      VisionInfoOut* info_out);

private:

    /**
        * Run detection for the given camera
    **/
    void detect_(CameraInfo::Camera whichCamera,
                 const VisionInfoIn* info_in, 
                 VisionInfoMiddle* info_middle,
                 VisionInfoOut* info_out);

    void configure(Blackboard* blackboard);

    NeuralNetwork::CompiledNN encoder;
    NeuralNetwork::CompiledNN classifier;
    NeuralNetwork::CompiledNN corrector;

    std::unique_ptr<NeuralNetwork::Model> encModel;
    std::unique_ptr<NeuralNetwork::Model> clModel;
    std::unique_ptr<NeuralNetwork::Model> corModel;

    std::size_t patchSize = 0;

    /**< Limit from which a ball is guessed. */
    float guessedThreshold = 0.7;

    /**< Limit from which a ball is accepted. */
    float acceptThreshold = 0.8;

    /**< Limit from which a ball is detected for sure. */
    float ensureThreshold = 0.9;

    /**< The kind of normalization used for patches. */
    NormalizationMode normalizationMode = normalizeBrightness;

    /**< The ratio of pixels ignored when determining the value range that is scaled to 0..255. */
    float normalizationOutlierRatio = 0.02;
    float ballAreaFactor = 3.5;
    bool useFloat = false;
    PatchUtilities::ExtractionMode extractionMode = PatchUtilities::fast;

    float apply(const Vector2i& ballSpot, Vector2f& ballPosition, float& predRadius, const VisionInfoIn* info_in, const CameraInfo& cameraInfo, ECImage& ecImage);
};
