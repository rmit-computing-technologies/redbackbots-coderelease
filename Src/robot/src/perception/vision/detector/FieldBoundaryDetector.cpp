
/**
 * @file FieldBoundaryDetector.cpp
 *
 * This file implements a module that calculates the field boundary
 * using a deep neural network (and subsequent line fitting).
 *
 * @author Arne Hasselbring
 * @author Yannik Meinken
 * @author Jonah Jaeger
 * @author Thorben Lorenzen
 * @author RedbackBots
 */


#include "perception/vision/detector/FieldBoundaryDetector.hpp"

#include "perception/vision/VisionInfoMiddle.hpp"
#include "types/geometry/Point.hpp"
#include "types/math/Eigen.hpp"
#include "utils/debug/Assert.hpp"
#include "utils/Logger.hpp"
#include "utils/SpatialUtilities.hpp"      // where does this come from? not in directory
#include "utils/math/LeastSquares.hpp"
#include "utils/ml/PatchUtilities.hpp"

#include <asmjit/asmjit.h>


#define MINIMUM_REQUIRED_SPOTS  3


FieldBoundaryDetector::FieldBoundaryDetector(Blackboard* blackboard, asmjit::JitRuntime* jitRuntime):
    Detector("FieldBoundaryDetector"),
    networkUpper(jitRuntime),
    networkLower(jitRuntime),
    modelUpper(nullptr),
    modelLower(nullptr)
{
    configure(blackboard);

    std::string modelDir = blackboard->config["vision.ml.modeldir"].as<std::string>();
    std::string modelUpperFile = modelDir + "/FieldBoundary/net.h5";
    std::string modelLowerFile = modelDir + "/FieldBoundary/net-uncertainty.h5";
    llog(INFO) << NDEBUG_LOGSYMB << "FieldBoundary Loading Upper Model from file " << modelUpperFile << std::endl;
    llog(INFO) << NDEBUG_LOGSYMB << "FieldBoundary Loading Lower Model from file " << modelLowerFile << std::endl;

    modelUpper = std::make_unique<NeuralNetwork::Model>(modelUpperFile);
    modelUpper->setInputUInt8(0);
    modelLower = std::make_unique<NeuralNetwork::Model>(modelLowerFile);
    modelLower->setInputUInt8(0);

    networkUpper.compile(*modelUpper);
    networkLower.compile(*modelLower);

    patchSizeUpper = Vector2i(networkUpper.input(0).dims(1), networkUpper.input(0).dims(0));
    patchSizeLower = Vector2i(networkLower.input(0).dims(1), networkLower.input(0).dims(0));

    auto testNetwork = [](NeuralNetwork::CompiledNN& network, Vector2i& patchSize) {
        ASSERT(network.valid());
        ASSERT(network.numOfInputs() == 1);
        ASSERT(network.numOfOutputs() == 1);
        ASSERT(network.input(0).rank() == 3);
        ASSERT(network.input(0).dims(2) == 1 || network.input(0).dims(2) == 3);

        ASSERT(network.output(0).rank() == 1 || network.output(0).rank() == 2);
        ASSERT(network.output(0).dims(0) == static_cast<unsigned>(patchSize.x()));
        ASSERT(network.output(0).rank() == 1 || network.output(0).dims(1) == 2);
    };
    testNetwork(networkUpper, patchSizeUpper);
    testNetwork(networkLower, patchSizeLower);

    llog(INFO) << NDEBUG_LOGSYMB << "FieldBoundary Models loaded and compiled" << std::endl;
}

FieldBoundaryDetector::~FieldBoundaryDetector() {
    modelUpper = nullptr;
    modelLower = nullptr;
}

void FieldBoundaryDetector::configure(Blackboard* blackboard) {

}

void FieldBoundaryDetector::resetMiddleInfo(VisionInfoMiddle* info_middle) {
}

void FieldBoundaryDetector::resetVisionOut(VisionInfoOut* info_out) {
    for (int i = 0; i != CameraInfo::Camera::NUM_CAMERAS; ++i) {
        info_out->fieldBoundary[i].reset();
    }
}

void FieldBoundaryDetector::detect_(const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << __PRETTY_FUNCTION__ << std::endl;

    // Top camera must run first, as the bottom camera is only computed if the top camera boundary can't be projected
    detect_(CameraInfo::Camera::top, info_in, info_middle, info_out);

    // TODO (TW): decide if lower field boundary is actually needed
    detect_(CameraInfo::Camera::bot, info_in, info_middle, info_out);
}

void FieldBoundaryDetector::detect_(CameraInfo::Camera whichCamera,
    const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << NDEBUG_LOGSYMB << "Detect with camera" << CameraInfo::enumCameraToString(whichCamera) << std::endl;

    // Setup references for the variables in this method based on the camera being used
    FieldBoundary& fieldBoundary = info_out->fieldBoundary[whichCamera];
    
    fieldBoundary.boundaryInImage.clear();
    fieldBoundary.boundaryOnField.clear();


    fieldBoundary.boundaryInImageUpperBound.clear();
    fieldBoundary.boundaryInImageLowerBound.clear();


    if (whichCamera == CameraInfo::Camera::top) {
        // llog(DEBUG) << "FieldBoundaryDetector::detect_:" << " Processing top camera" << std::endl;
        if (networkUpper.valid()) {
            // llog(DEBUG) << "\t network is valid" << std::endl;
            const CameraImage* upperImage = info_in->image[CameraInfo::Camera::top];
            const CameraInfo& upperInfo = info_in->cameraInfo[CameraInfo::Camera::top];
            FieldBoundary& otherFieldBoundary = info_out->fieldBoundary[CameraInfo::Camera::bot];

            std::vector<Spot> spots;
            predictSpots(networkUpper, patchSizeUpper, upperImage, upperInfo, info_in, spots);
            bool odd = boundaryIsOdd(spots);
            if (odd && !otherFieldBoundary.extrapolated) {
                projectPrevious(fieldBoundary, otherFieldBoundary, upperInfo, info_in);
            } else {
                validatePrediction(fieldBoundary, spots);
            }
            fieldBoundary.odd = odd;
        }
    } else  if (whichCamera == CameraInfo::Camera::bot) {
        if (networkLower.valid()) {
            // llog(DEBUG) << "NetworkLower is valid. Running predictions." << std::endl;
            FieldBoundary& otherFieldBoundary = info_out->fieldBoundary[CameraInfo::Camera::top];
            const CameraImage* lowerImage = info_in->image[CameraInfo::Camera::bot];
            const CameraInfo& lowerInfo = info_in->cameraInfo[CameraInfo::Camera::bot];
            if(otherFieldBoundary.odd || otherFieldBoundary.extrapolated || otherFieldBoundary.boundaryInImage.empty()) {
                std::vector<Spot> spots;
                predictSpots(networkLower, patchSizeLower, lowerImage, lowerInfo, info_in, spots);

                // llog(DEBUG) << "PREDICTED SPOTS FOR BOTTOM CAMERA" << std::endl;
                for (auto spot : spots) {
                    //llog(DEBUG) << "Spot in image: (x=" << spot.inImage.x() << ", y=" << spot.inImage.y() << ")" << std::endl;
                    //llog(DEBUG) << "Spot on field: (x=" << spot.onField.x() << ", y=" << spot.onField.y() << ")" << std::endl;

                    fieldBoundary.boundaryInImageUpperBound.emplace_back(spot.inImage.x(), spot.inImage.y() + spot.uncertanty);
                    fieldBoundary.boundaryInImageLowerBound.emplace_back(spot.inImage.x(), spot.inImage.y() - spot.uncertanty);

                    //llog(DEBUG) << "LBound size: " << fieldBoundary.boundaryInImageLowerBound.size() << std::endl;

                }
                // llog(DEBUG) << "DONE.\n\n" << std::endl;

                if (otherFieldBoundary.boundaryInImage.size() > 1 && boundaryIsOdd(spots)){
                    // llog(DEBUG) << "Projecting Previous" << std::endl;
                    projectPrevious(fieldBoundary, otherFieldBoundary, lowerInfo, info_in);
                } else {
                    // llog(DEBUG) << "Validating Predictions" << std::endl;
                    validatePrediction(fieldBoundary, spots);
                } 
            } else {
                // llog(DEBUG) << "NetworkLower is invalid. ProjectingPrevious" << std::endl;
                projectPrevious(fieldBoundary, otherFieldBoundary, lowerInfo, info_in);
            }
        }
    }
    fieldBoundary.isValid = fieldBoundary.boundaryInImage.size() > 1;

    if(!fieldBoundary.isValid) {
        // llog(DEBUG) << "Field Boundary is invalid" << std::endl;
        fieldBoundary.boundaryInImage.clear();
        fieldBoundary.boundaryOnField.clear();

        fieldBoundary.boundaryInImageUpperBound.clear();
        fieldBoundary.boundaryInImageLowerBound.clear();
    }
}


FieldBoundaryDetector::Spot::Spot(const Vector2i& inImage, const Vector2f& onField, const float u) : 
    inImage(inImage), 
    onField(onField), 
    uncertanty(u)
{}

void FieldBoundaryDetector::predictSpots(NeuralNetwork::CompiledNN& network, Vector2i& patchSize,
                                         const CameraImage* cameraImage, const CameraInfo& cameraInfo,
                                         const VisionInfoIn* info_in,
                                         std::vector<Spot>& spots) {
    unsigned char* input = reinterpret_cast<std::uint8_t*>(network.input(0).data());

    if(network.input(0).dims(2) == 1) {
        // llog(DEBUG) << "network input(0) dims(2) is 1" << std::endl;
        PatchUtilities::extractInput<std::uint8_t, true>(*cameraImage, patchSize, input);
    } else {
        PatchUtilities::extractInput<std::uint8_t, false>(*cameraImage, patchSize, input);
    }

    network.apply();
    const float* output = network.output(0).data();

    const unsigned int xScale = cameraInfo.width / patchSize(0);
    const unsigned int stepSize = network.output(0).rank() == 2 ? 2 : 1;
    for (int x = 0, idx = 0; x < patchSize(0); ++x, idx += static_cast<int>(stepSize)) {
        const Vector2f spotInImage(x * xScale + xScale / 2, std::max(0.f, std::min(output[idx], 1.f)) * static_cast<float>(cameraInfo.height - 1));
        
        float uncertainty = 0;
        if(network.output(0).rank() == 2) {
            uncertainty = 1.f / (output[idx + 1] * output[idx + 1]) * static_cast<float>(cameraInfo.height - 1);
        }

        //  llog(DEBUG) << "uncertainty: " << uncertainty << std::endl;

        // TODO: revisit this. should be headrel or not.
        PointF spotOnField = info_in->kinematicPose.imageToRobotXY(spotInImage, cameraInfo);
        // PointF spotOnField = info_in->kinematicPose.headRelImageToRobotXY(spotInImage, cameraInfo);

        if (SpatialUtilities::possiblyOnFieldRRXY(spotOnField)
            && spotOnField.squaredNorm() >= minDistanceSqr) {
            spots.emplace_back(spotInImage.cast<int>(), spotOnField, uncertainty);
        }
    }
}

bool FieldBoundaryDetector::boundaryIsOdd(const std::vector<Spot>& spots) const {
    // With less than 3 points not usable
    if (spots.size() < MINIMUM_REQUIRED_SPOTS) {
        return true;
    }

    // Find the lowest point of the three left/right most points
    int yMaxBorder = 0;
    for(size_t i = 0; i < 3; i++) {
        if(spots[i].inImage.y() > yMaxBorder)
        yMaxBorder = spots[i].inImage.y();

        if(spots[spots.size() - 1 - i].inImage.y() > yMaxBorder)
        yMaxBorder = spots[spots.size() - 1 - i].inImage.y();
    }

    // For each triple of following points sum the difference from the middle point 
    //     to the line between the first and last point.
    int toLowSum = 0;
    float uncertaintySum = 0;
    float sum = 0;
    float gradient;
    int num = 0;
    for (size_t i = 0; i < spots.size(); i++) {
        // Only if the point is not at the top of the image
        if (spots[i].inImage.y() > top) {
            num++;

            // Ff the point is below the lower end
            if (spots[i].inImage.y() > yMaxBorder) {
                toLowSum += spots[i].inImage.y() - yMaxBorder;
            }

            uncertaintySum += spots[i].uncertanty;

            if (i + 2 >= spots.size()) {
                continue;
            }

            // Compute the discrepancy of the middle point to the line between the first and third
            gradient = static_cast<float>(spots[i].inImage.y() - spots[i + 2].inImage.y()) / static_cast<float>(spots[i].inImage.x() - spots[i + 2].inImage.x());
            sum += std::abs((spots[i].inImage.y() + gradient * (spots[i + 1].inImage.x() - spots[i].inImage.x())) - spots[i + 1].inImage.y());
        }
    }

    // If one of the limits is violated
    if ((static_cast<float>(num) / static_cast<float>(spots.size()) < nonTopPoints) 
        || uncertaintySum / static_cast<float>(num) > uncertaintyLimit 
        || toLowSum / num > maxPointsUnderBorder 
        || sum / static_cast<float>(num) > threshold) {
        return true;
    }

    // If we get to here, boundary is OK, is NOT odd
    return false;
}

void FieldBoundaryDetector::projectPrevious(FieldBoundary& fieldBoundary, FieldBoundary& otherFieldBoundary,
                                            const CameraInfo& cameraInfo,
                                            const VisionInfoIn* info_in) {
    for(Vector2f spotOnField : otherFieldBoundary.boundaryOnField) {
        Point spotInImage = info_in->kinematicPose.robotToImageXY(spotOnField, cameraInfo);

        // TODO: CHeck if the transposed spot could be a bad pixel (ie "behind" the robot)
        fieldBoundary.boundaryInImage.emplace_back(spotInImage);
        fieldBoundary.boundaryOnField.emplace_back(spotOnField);
    }
    fieldBoundary.extrapolated = true;
}

void FieldBoundaryDetector::validatePrediction(FieldBoundary& fieldBoundary, std::vector<Spot>& spots) {
    /*
     * NOTE: (TW): The original B-Human implementation has fitting options by which to validate
     *             The detected boundary using methods such as RANSAC.
     *             This isn't used by default, and is likely for original testing.
     *             Should this be required, revisit the 2023 B-Human implementation of this method
    */
    
    fieldBoundary.isValid = spots.size() >= minNumberOfSpots;
    if (fieldBoundary.isValid) {
        for (const Spot& spot : spots) {
            fieldBoundary.boundaryInImage.emplace_back(spot.inImage);
            fieldBoundary.boundaryOnField.emplace_back(spot.onField);
        }
        fieldBoundary.extrapolated = false;
    }
}
