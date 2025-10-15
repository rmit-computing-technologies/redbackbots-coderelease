/**
 * @file PenaltyMarkMiddleInfo.cpp
 *
 * This file implements a module that detects penalty marks in images with a neural network.
 *
 * @author Simon Werner
 * @author Finn Ewers
 * @author Thomas Röfer
 * @author RedbackBots
 */

#include "perception/vision/middleinfo/PenaltyMarkMiddleInfo.hpp"

#include "perception/vision/VisionInfoMiddle.hpp"
#include "utils/math/IISC.hpp"

PenaltyMarkMiddleInfo::PenaltyMarkMiddleInfo(Blackboard* blackboard, asmjit::JitRuntime* jitRuntime):
  Detector("PenaltyMarkMiddleInfo"),
  network(jitRuntime),
  model(nullptr)
{
    configure(blackboard);

    std::string modelDir = blackboard->config["vision.ml.modeldir"].as<std::string>();
    std::string modelFile = modelDir + "/PenaltyMark/penalty_model.h5";
    llog(INFO) << NDEBUG_LOGSYMB << "PenaltyMark Loading Model from file " << modelFile << std::endl;

    model = std::make_unique<NeuralNetwork::Model>(modelFile);
    network.compile(*model);

    llog(INFO) << NDEBUG_LOGSYMB << "PenaltyMarkMiddleInfo loaded" << std::endl;
}

PenaltyMarkMiddleInfo::~PenaltyMarkMiddleInfo() {

}

void PenaltyMarkMiddleInfo::configure(Blackboard* blackboard) {

}

void PenaltyMarkMiddleInfo::resetMiddleInfo(VisionInfoMiddle* info_middle) {
    info_middle->penaltyMarkInfo[CameraInfo::Camera::top].reset();
    info_middle->penaltyMarkInfo[CameraInfo::Camera::bot].reset();
}

void PenaltyMarkMiddleInfo::resetVisionOut(VisionInfoOut* info_out) {
}

void PenaltyMarkMiddleInfo::detect_(const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << __PRETTY_FUNCTION__ << std::endl;
    // llog(INFO) << NDEBUG_LOGSYMB << "tick PenaltyMarks" << std::endl;

    // Top camera
    detect_(CameraInfo::Camera::top, info_in, info_middle, info_out);

    // Bottom camera
    detect_(CameraInfo::Camera::bot, info_in, info_middle, info_out);
}

void PenaltyMarkMiddleInfo::detect_(CameraInfo::Camera whichCamera,
    const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {

    const CameraInfo& cameraInfo = info_in->cameraInfo[whichCamera];
    ECImage& ecImage = info_middle->ecImage[whichCamera];
    PenaltyMarkRegions& penaltyMarkRegions = info_middle->penaltyMarkRegions[whichCamera];

    PenaltyMarkInfo& penaltyMarkInfo = info_middle->penaltyMarkInfo[whichCamera];

    // Special handling for penalty shootout: the penalty mark is where the ball is.
    /*
    if(theGameState.isPenaltyShootout())
    {
        if(theBallPercept.status == BallPercept::Status::seen)
        {
        thePenaltyMarkPercept.positionInImage = theBallPercept.positionInImage.cast<int>();

        // Todo use covariance of ball percept
        if(theMeasurementCovariance.transformWithCov(theBallPercept.positionInImage, theBallSpecification.radius, thePenaltyMarkPercept.positionOnField,
                                                    thePenaltyMarkPercept.covarianceOnField))
            thePenaltyMarkPercept.wasSeen = true;
        return;
        }
    }
    */

    if(!network.valid() || penaltyMarkRegions.regions.empty()) {
        return;
    }

    for(const Boundaryi& region : penaltyMarkRegions.regions)
    {
        //extract patch
        float width, height;
        const Vector2f center(region.x.getCenter(), region.y.getCenter());
        if(!IISC::calculateImagePenaltyMeasurementsByCenter(center, width, height, info_in, cameraInfo)) {
            continue;
        }

        const int inputSize = static_cast<int>(std::max(width, height) * 1.7f);
        PatchUtilities::extractPatch(center.cast<int>(), Vector2i(inputSize, inputSize), Vector2i(32, 32), ecImage.grayscaled, network.input(0).data());

        network.apply();

        //do something with prediction
        const float first = network.output(0)[0];
        const float second = network.output(0)[1];
        const float confidence = second - first;

        // llog(DEBUG) << "    - confidence : " << confidence << std::endl;

        if(first < second && confidence > threshold)
        // && theMeasurementCovariance.transformWithCov(center, 0.f, thePenaltyMarkPercept.positionOnField, thePenaltyMarkPercept.covarianceOnField))
        {
            penaltyMarkInfo.positionInImage = center.cast<int>(); // todo: estimate actual center
            penaltyMarkInfo.wasSeen = true;
            break;
        }
    }
}