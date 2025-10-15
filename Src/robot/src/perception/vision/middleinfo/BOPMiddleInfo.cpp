/**
 * @file BOPMiddleInfo.cpp
 *
 * This file implements a module that runs a neural network on a full image
 * to detect balls, obstacles and penalty marks.
 *
 * @author Arne Hasselbring
 * @author RedbackBots
 */

#include "perception/vision/middleinfo/BOPMiddleInfo.hpp"

#include "perception/vision/VisionInfoMiddle.hpp"
#include "utils/math/IISC.hpp"
#include "utils/SpatialUtilities.hpp" 
#include "utils/debug/Assert.hpp"

#include <algorithm>
#include <cstring>
#include <type_traits>

#include <asmjit/asmjit.h>

BOPMiddleInfo::BOPMiddleInfo(Blackboard* blackboard, asmjit::JitRuntime* jitRuntime):
    Detector("BOPMiddleInfo"),
    network(jitRuntime),
    model(nullptr)
{
    configure(blackboard);

    std::string modelDir = blackboard->config["vision.ml.modeldir"].as<std::string>();
    std::string modelFile = modelDir + "/BOP/net.h5";
    llog(INFO) << NDEBUG_LOGSYMB << "BOP Loading Model from file " << modelFile << std::endl;

    model = std::make_unique<NeuralNetwork::Model>(modelFile);
    model->setInputUInt8(0);
    NeuralNetwork::CompilationSettings settings;
    network.compile(*model, settings);

    ASSERT(network.valid());

    ASSERT(network.numOfInputs() == 1);
    ASSERT(network.numOfOutputs() == 1);

    ASSERT(network.input(0).rank() == 3);
    inputSize = Vector2i(network.input(0).dims(1), network.input(0).dims(0));
    ASSERT(network.input(0).dims(2) == 2);

    ASSERT(network.output(0).rank() == 3);
    outputSize = Vector2i(network.output(0).dims(1), network.output(0).dims(0));
    ASSERT(network.output(0).dims(2) == numOfChannels);

    ASSERT(inputSize.x() % outputSize.x() == 0);
    ASSERT(inputSize.y() % outputSize.y() == 0);
    scale = Vector2i(inputSize.x() / outputSize.x(), inputSize.y() / outputSize.y());

    llog(INFO) << NDEBUG_LOGSYMB << "BOP Models loaded and compiled" << std::endl;
}

BOPMiddleInfo::~BOPMiddleInfo() {
    model = nullptr;
}

void BOPMiddleInfo::configure(Blackboard* blackboard) {

}

void BOPMiddleInfo::resetMiddleInfo(VisionInfoMiddle* info_middle) {
    info_middle->ballSpots[CameraInfo::Camera::bot].ballSpots.clear();
    info_middle->ballSpots[CameraInfo::Camera::bot].firstSpotIsPredicted = false;

    info_middle->penaltyMarkRegions[CameraInfo::Camera::bot].regions.clear();
}

void BOPMiddleInfo::resetVisionOut(VisionInfoOut* info_out) {
}

void BOPMiddleInfo::detect_(const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << __PRETTY_FUNCTION__ << std::endl;
    // llog(INFO) << NDEBUG_LOGSYMB << "tick BOP" << std::endl;

    // (MO) BOP runs only on bottom camera
    // detect_(CameraInfo::Camera::top, info_in, info_middle, info_out);

    // Bottom camera
    detect_(CameraInfo::Camera::bot, info_in, info_middle, info_out);
}

void BOPMiddleInfo::detect_(CameraInfo::Camera whichCamera,
    const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << NDEBUG_LOGSYMB << "Detect with camera" << CameraInfo::enumCameraToString(whichCamera) << std::endl;

    // Setup references for the variables in this method based on the camera being used
    const CameraImage* cameraImage = info_in->image[whichCamera];
    const CameraInfo& cameraInfo = info_in->cameraInfo[whichCamera];

    BallSpots& ballSpots = info_middle->ballSpots[whichCamera];
    PenaltyMarkRegions& penaltyMarkRegions = info_middle->penaltyMarkRegions[whichCamera];
    SegmentedObstacleImage& segmentedObstacleImage = info_middle->segmentedObstacleImage;

    updateBallSpots(ballSpots, cameraImage, cameraInfo);
    updatePenaltyMarkRegions(penaltyMarkRegions, info_in, cameraImage, cameraInfo);
    updateSegmentedObstacles(segmentedObstacleImage, cameraImage, cameraInfo);
}

bool BOPMiddleInfo::apply(const CameraImage* cameraImage, const CameraInfo& cameraInfo)
{
    if(lastPrediction == cameraImage->timestamp) {
        // llog(DEBUG) << NDEBUG_LOGSYMB << "BOP: last timestamp is current image" << std::endl;
        return true;
    }
    // llog(DEBUG) << NDEBUG_LOGSYMB << "BOP: different timestamps" << std::endl;

    if(cameraInfo.width != inputSize.x() || cameraInfo.height != inputSize.y()) {
        // llog(DEBUG) << NDEBUG_LOGSYMB << "BOP: invalid image" << std::endl;
        return false;
    }

    static_assert(std::is_same<CameraImage::PixelType, PixelTypes::YUYVPixel>::value);
    // TODO: CompiledNN should be able to take an external buffer as input (but this is more complicated than one could think).
    // In the meantime, one could directly convert to float in this copy operation using SSE.
    std::memcpy(reinterpret_cast<std::uint8_t*>(network.input(0).data()), (*cameraImage)[0], inputSize.x() * inputSize.y() * 2);
    network.apply();

    lastPrediction = cameraImage->timestamp;

    return true;
}

void BOPMiddleInfo::updateBallSpots(BallSpots& ballSpots, const CameraImage* cameraImage, const CameraInfo& cameraInfo)
{
    if(!apply(cameraImage, cameraInfo)) {
        return;
    }

    const float* data = network.output(0).data();

    Vector2i maxPos;
    float max = ballThreshold;
    for(int y = 0; y < outputSize.y(); ++y) 
    {
        for(int x = 0; x < outputSize.x(); ++x)
        {
            const float f = *data;
            if(f > max)
            {
                max = f;
                maxPos.x() = x;
                maxPos.y() = y;
            }
            data += numOfChannels;
        }
    }

    /*
    int xlog = maxPos.x() * scale.x() + scale.x() / 2;
    int ylog = maxPos.y() * scale.y() + scale.y() / 2;
    llog(DEBUG) << "BALL SPOT" << std::endl;
    llog(DEBUG) << "    - x: " << xlog << std::endl;
    llog(DEBUG) << "    - y: " << ylog << std::endl;
    llog(DEBUG) << "    - max: " << max << std::endl;
    */

    if(max > ballThreshold)
        ballSpots.addBallSpot(maxPos.x() * scale.x() + scale.x() / 2, maxPos.y() * scale.y() + scale.y() / 2);
    }

void BOPMiddleInfo::updatePenaltyMarkRegions(PenaltyMarkRegions& penaltyMarkRegions, const VisionInfoIn* info_in, const CameraImage* cameraImage, const CameraInfo& cameraInfo)
{
    if(!apply(cameraImage, cameraInfo)) {
        return;
    }

    const float* data = network.output(0).data() + penaltyMarkIndex;

    Vector2i maxPos;
    float max = penaltyMarkThreshold;
    for(int y = 0; y < outputSize.y(); ++y) {
        for(int x = 0; x < outputSize.x(); ++x)
        {
            const float f = *data;
            // llog(DEBUG) << "f: " << f << std::endl;
            if(f > max)
            {
                max = f;
                maxPos.x() = x;
                maxPos.y() = y;
            }
            data += numOfChannels;
        }
    }

    // llog(DEBUG) << "max: " << max << std::endl;
    if(max > penaltyMarkThreshold)
    {
        const Vector2i center(maxPos.x() * scale.x() + scale.x() / 2, maxPos.y() * scale.y() + scale.y() / 2);
        float expectedWidth;
        float expectedHeight;
        static constexpr float sizeToleranceRatio = 0.5f;
        static constexpr int blockSizeX = 16;
        static constexpr int blockSizeY = 16;
        if(!IISC::calculateImagePenaltyMeasurementsByCenter(center.cast<float>(), expectedWidth, expectedHeight, info_in, cameraInfo)) {
            // llog(DEBUG) << "BOP: IISC failed" << std::endl;
            return;
        }

        // llog(DEBUG) << "Pre-ymin: " << center.y() - scale.y() / blockSizeY * blockSizeY << std::endl;
        // llog(DEBUG) << "Pre-ymax: " << center.y() + scale.y() / blockSizeY * blockSizeY << std::endl;

        Boundaryi region(Rangei((center.x() - scale.x()) / blockSizeX * blockSizeX,
                                (center.x() + scale.x()) / blockSizeX * blockSizeX),
                        Rangei((center.y() - scale.y()) / blockSizeY * blockSizeY,
                                (center.y() + scale.y()) / blockSizeY * blockSizeY));
        const int xExtent = (static_cast<int>(expectedWidth * (1.f + sizeToleranceRatio) / 2.f) + blockSizeX - 1) / blockSizeX * blockSizeX;
        const int yExtent = (static_cast<int>(expectedHeight * (1.f + sizeToleranceRatio) / 2.f) + blockSizeY - 1) / blockSizeY * blockSizeY;
        const Boundaryi cnsRegion(Rangei(std::max(0, region.x.min - xExtent),
                                        std::min(cameraInfo.width, region.x.max + xExtent)),
                                Rangei(std::max(0, region.y.min - yExtent),
                                        std::min(cameraInfo.height, region.y.max + yExtent)));
        region = Boundaryi(Rangei(cnsRegion.x.min + xExtent, cnsRegion.x.max - xExtent),
                        Rangei(cnsRegion.y.min + yExtent, cnsRegion.y.max - yExtent));
        if(region.x.min >= region.x.max || region.y.min >= region.y.max){
            // llog(DEBUG) << "Region is invalid" << std::endl;
            return;
        }

        // llog(DEBUG) << "REGION" << std::endl;
        // llog(DEBUG) << "    - xmin: " << region.x.min << std::endl;
        // llog(DEBUG) << "    - xmax: " << region.x.max << std::endl;
        // llog(DEBUG) << "    - ymin: " << region.y.min << std::endl;
        // llog(DEBUG) << "    - ymax: " << region.y.max << std::endl;

        penaltyMarkRegions.regions.push_back(region);
    }
}

void BOPMiddleInfo::updateSegmentedObstacles(SegmentedObstacleImage& segmentedObstacleImage, const CameraImage* cameraImage, const CameraInfo& cameraInfo)
{
    if(!apply(cameraImage, cameraInfo)) {
        segmentedObstacleImage.obstacle.setResolution(0, 0);
        return;
    }

    segmentedObstacleImage.obstacle.setResolution(outputSize.x(), outputSize.y());
    const float* data = network.output(0).data() + obstaclesIndex;
    for(int y = 0; y < outputSize.y(); ++y) {
        for(int x = 0; x < outputSize.x(); ++x, data += numOfChannels) {
            segmentedObstacleImage.obstacle[y][x] = static_cast<unsigned char>(std::max(0.f, std::min(*data * 255.f, 255.f)));
        }
    }
}