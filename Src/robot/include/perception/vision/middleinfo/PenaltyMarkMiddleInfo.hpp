/**
 * @file PenaltyMarkMiddleInfo.hpp
 *
 * This file declares a module that detects penalty marks in images with a neural network.
 *
 * @author Simon Werner
 * @author Finn Ewers
 * @author RedbackBots
 */

#pragma once

#include "perception/vision/Detector.hpp"

#include "blackboard/Blackboard.hpp"
#include "utils/ml/PatchUtilities.hpp"
#include "utils/defs/FieldDefinitions.hpp"
#include "types/camera/CameraInfo.hpp"
#include "types/vision/ECImage.hpp"
#include "types/BallInfo.hpp"
#include "types/vision/PenaltyMarkInfo.hpp"

#include <CompiledNN.h>

class PenaltyMarkMiddleInfo : public Detector {
public:
    PenaltyMarkMiddleInfo(Blackboard* blackboard, asmjit::JitRuntime* jitRuntime);
    virtual ~PenaltyMarkMiddleInfo();

    // Resets
    virtual void resetMiddleInfo(VisionInfoMiddle* info_middle);
    virtual void resetVisionOut(VisionInfoOut* info_out);

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

    NeuralNetwork::CompiledNN network;
    std::unique_ptr<NeuralNetwork::Model> model;

    static constexpr float threshold = 8.f;
};