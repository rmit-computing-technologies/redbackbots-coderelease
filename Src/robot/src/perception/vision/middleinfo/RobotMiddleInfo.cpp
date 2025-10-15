/**
 * @file RobotMiddleInfo.cpp
 *
 * This file implements a module that detects SPL robots in an image using a neural network.
 *
 * @author Kelke van Lessen
 * @author Lukas Malte Monnerjahn
 * @author Fynn Boese
 * @author Bernd Poppinga
 * @author RedbackBots
 */

#include "perception/vision/middleinfo/RobotMiddleInfo.hpp"

#include "perception/vision/VisionInfoMiddle.hpp"
#include "perception/vision/VisionInfoOut.hpp"
#include "perception/vision/other/JerseyClassifier.hpp"
#include "utils/SpatialUtilities.hpp" 
#include "utils/ml/PatchUtilities.hpp"
#include "utils/ml/Resize.hpp"
#include "utils/math/basic_maths.hpp"
#include "utils/debug/Assert.hpp"

#include <asmjit/asmjit.h>

RobotMiddleInfo::RobotMiddleInfo(Blackboard* blackboard, asmjit::JitRuntime* jitRuntime):
    Detector("RobotMiddleInfo"),
    network(jitRuntime),
    model(nullptr)
{
    configure(blackboard);

    std::string modelDir = blackboard->config["vision.ml.modeldir"].as<std::string>();
    std::string modelFile = modelDir + "/RobotDetector/4_anchor_boxes_model_no_activation_20230629-220730.onnx";
    llog(INFO) << NDEBUG_LOGSYMB << "RobotMiddleInfo Loading Model from file " << modelFile << std::endl;

    NeuralNetwork::CompilationSettings settings;
    settings.useExpApproxInSigmoid = false;
    settings.useExpApproxInTanh = false;
    NeuralNetworkONNX::CompilationSettings onnxSettings;

    networkParameters.inputHeight = 60;
    networkParameters.inputWidth = 80;
    networkParameters.inputChannels = 3;
    networkParameters.outputHeight = 8;
    networkParameters.outputWidth = 10;
    networkParameters.outputAnchors = 4;
    networkParameters.paramsPerAnchor = 5;
    networkParameters.predictFallen = false;
    networkParameters.confidenceIndex = 0;
    networkParameters.yMidIndex = 1;
    networkParameters.xMidIndex = 2;
    networkParameters.heightIndex = 3;
    networkParameters.widthIndex = 4;
    networkParameters.fallenClassIndex = -1;
    networkParameters.anchors = {
        {0.09658511133978298, 0.2370820816605168},
        {0.14000417622375075, 0.4056380034479811},
        {0.2177281673006002, 0.7012438628796819},
        {0.3954677177611361, 0.9756357911017483}
    };
    networkParameters.sizeConversionFactor = 8.0;

    ASSERT(networkParameters.inputChannels == 1 || networkParameters.inputChannels == 3); // single channel grayscale image or three channel YUV image

    model = std::make_unique<NeuralNetworkONNX::Model>(modelFile);
    model->setInputUInt8(0); // This converts the uint8 image to floats for the model
    network.compile(*model, onnxSettings);
    
    ASSERT(network.numOfInputs() == 1);
    ASSERT(network.input(0).rank() == 3);
    ASSERT(networkParameters.inputHeight == network.input(0).dims(HEIGHT_SHAPE_INDEX));
    ASSERT(networkParameters.inputWidth == network.input(0).dims(WIDTH_SHAPE_INDEX));
    inputImageSize = Vector2i(networkParameters.inputWidth, networkParameters.inputHeight);
    ASSERT(networkParameters.inputChannels == network.input(0).dims(IMAGE_CHANNELS_INDEX));
    ASSERT(network.numOfOutputs() == 1);
    ASSERT(network.output(0).rank() == 4);
    ASSERT(networkParameters.outputHeight == network.output(0).dims(HEIGHT_SHAPE_INDEX));
    ASSERT(networkParameters.outputWidth == network.output(0).dims(WIDTH_SHAPE_INDEX));
    ASSERT(networkParameters.outputAnchors == network.output(0).dims(ANCHOR_SHAPE_INDEX));
    ASSERT(networkParameters.paramsPerAnchor == network.output(0).dims(BOX_SHAPE_INDEX));

    llog(INFO) << NDEBUG_LOGSYMB << "Robot Detector Models loaded and compiled" << std::endl;
}

RobotMiddleInfo::~RobotMiddleInfo() {
    model = nullptr;
    blackboard = nullptr;
}

void RobotMiddleInfo::configure(Blackboard* blackboard) {
    this->blackboard = blackboard;
}

void RobotMiddleInfo::resetMiddleInfo(VisionInfoMiddle* info_middle) {
    info_middle->robotsImage[CameraInfo::Camera::top].obstacles.clear();

    info_middle->obstaclesData[CameraInfo::Camera::top].scanScores.clear();
}

void RobotMiddleInfo::resetVisionOut(VisionInfoOut* info_out) {
}

void RobotMiddleInfo::detect_(const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << __PRETTY_FUNCTION__ << std::endl;

    // Top camera
    detect_(CameraInfo::Camera::top, info_in, info_middle, info_out);

    // Bottom camera - done by RobotLowerMiddleInfo
    // detect_(CameraInfo::Camera::bot, info_in, info_middle, info_out);
}

void RobotMiddleInfo::detect_(CameraInfo::Camera whichCamera,
    const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {

    const CameraInfo& cameraInfo = info_in->cameraInfo[whichCamera];
    FieldBoundary& fieldBoundary = info_out->fieldBoundary[whichCamera];
    ECImage& ecImage = info_middle->ecImage[whichCamera];
    RobotObstaclesField& robotsField = info_middle->robotsField[whichCamera];
    RobotObstaclesImage& robotsImage = info_middle->robotsImage[whichCamera];
    RobotObstaclesIncomplete& obstaclesData = info_middle->obstaclesData[whichCamera];
    RobotObstaclesIncomplete& otherObstaclesData = info_middle->obstaclesData[CameraInfo::Camera::bot];
    
    updateRobotObstaclesField(info_in, cameraInfo, fieldBoundary, ecImage, robotsField, obstaclesData, otherObstaclesData);
    updateRobotObstaclesImage(robotsImage);
}

void RobotMiddleInfo::updateRobotObstaclesImage(RobotObstaclesImage& obstaclesImage) {
    obstaclesImage.obstacles = obstaclesUpper;
}

void RobotMiddleInfo::updateRobotObstaclesField(const VisionInfoIn* info_in, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary, ECImage& ecImage, RobotObstaclesField& robotsField, RobotObstaclesIncomplete& obstaclesData, RobotObstaclesIncomplete& otherObstaclesData) {
    std::vector<RobotObstaclesImage::Obstacle>& obstacles = obstaclesUpper;
    obstacles.clear();
    robotsField.obstacles.clear();
    obstaclesData.incompleteObstacles.clear();

    if(!fieldBoundary.isValid || !ecImage.grayscaled.width || !ecImage.grayscaled.height) {
        //llog(INFO) << NDEBUG_LOGSYMB << "   - invalid field boundary" << std::endl;
        return;
    }

    if(cameraInfo.camera == CameraInfo::Camera::top) {
        extractImageObstaclesFromNetwork(obstacles, cameraInfo, ecImage, fieldBoundary);
    }
    else{
        ASSERT(xyStep < ecImage.grayscaled.width && xyStep > 0 && xyStep < ecImage.grayscaled.height && xyStep > 0);

        std::vector<std::vector<Region>> regions(xyRegions, std::vector<Region>(xyRegions, Region()));
        scanImage(regions, cameraInfo, fieldBoundary, ecImage);
        classifyRegions(regions, ecImage);
        discardHomogeneousAreas(regions);
        dbScan(regions, obstacles, cameraInfo, ecImage);
    }

    mergeObstacles(robotsField, obstaclesData, obstacles, info_in, cameraInfo, ecImage, otherObstaclesData);
}

void RobotMiddleInfo::extractImageObstaclesFromNetwork(std::vector<RobotObstaclesImage::Obstacle>& obstacles, const CameraInfo& cameraInfo, ECImage& ecImage, FieldBoundary& fieldBoundary) {
    if(!network.valid()) {
        // llog(INFO) << NDEBUG_LOGSYMB << "   - invalid network" << std::endl;
        return;
    }

    LabelImage labelImage;

    if(networkParameters.inputChannels == 1) {
        applyGrayscaleNetwork(ecImage);
    }
    else {
        applyColorNetwork(ecImage);
    }

    boundingBoxes(labelImage, cameraInfo, fieldBoundary);
    labelImage.nonMaximumSuppression(nonMaximumSuppressionIoUThreshold);
    labelImage.bigBoxSuppression();

    for(const LabelImage::Annotation& box : labelImage.annotations) {
        obstacles.emplace_back();
        obstacles.back().top = static_cast<int>(box.upperLeft.y());
        obstacles.back().bottom = static_cast<int>(box.lowerRight.y());
        obstacles.back().left = static_cast<int>(box.upperLeft.x());
        obstacles.back().right = static_cast<int>(box.lowerRight.x());
        obstacles.back().confidence = box.confidence;
        obstacles.back().fallen = box.fallen;
        obstacles.back().distance = box.distance;
        obstacles.back().bottomFound = true;
    }
}

void RobotMiddleInfo::fillGrayscaleThumbnail(ECImage& ecImage) {
  const auto scale = static_cast<unsigned int>(std::round(std::log2(ecImage.grayscaled.width / networkParameters.inputWidth)));
  ASSERT(ecImage.grayscaled.width == static_cast<unsigned>(networkParameters.inputWidth) << scale);
  ASSERT(ecImage.grayscaled.height == static_cast<unsigned>(networkParameters.inputHeight) << scale);

  // Can't shrink directly to NN input because it needs a larger buffer :-(
  Resize::shrinkY(scale, ecImage.grayscaled, grayscaleThumbnail);
  ASSERT(networkParameters.inputWidth == grayscaleThumbnail.width);
  ASSERT(networkParameters.inputHeight == grayscaleThumbnail.height);
}

void RobotMiddleInfo::fillChromaThumbnails(ECImage& ecImage) {
  const auto scale = static_cast<unsigned int>(std::round(std::log2(ecImage.blueChromaticity.width / networkParameters.inputWidth)));
  ASSERT(scale == static_cast<unsigned int>(std::round(std::log2(ecImage.redChromaticity.width / networkParameters.inputWidth))));
  ASSERT(ecImage.blueChromaticity.width == ecImage.redChromaticity.width);
  ASSERT(ecImage.blueChromaticity.width == static_cast<unsigned>(networkParameters.inputWidth) << scale);
  ASSERT(ecImage.blueChromaticity.height == ecImage.redChromaticity.height);
  ASSERT(ecImage.blueChromaticity.height == static_cast<unsigned>(networkParameters.inputHeight) << scale);
  Resize::shrinkY(scale, ecImage.blueChromaticity, redChromaThumbnail);
  Resize::shrinkY(scale, ecImage.redChromaticity, blueChromaThumbnail);
  ASSERT(networkParameters.inputWidth == redChromaThumbnail.width);
  ASSERT(networkParameters.inputWidth == blueChromaThumbnail.width);
  ASSERT(networkParameters.inputHeight == redChromaThumbnail.height);
  ASSERT(networkParameters.inputHeight == blueChromaThumbnail.height);
}

void RobotMiddleInfo::applyGrayscaleNetwork(ECImage& ecImage) {
    ASSERT(networkParameters.inputChannels == 1);
    fillGrayscaleThumbnail(ecImage);

    // Copy image into input of the model
    std::memcpy(reinterpret_cast<unsigned char*>(network.input(0).data()), grayscaleThumbnail[0], grayscaleThumbnail.width * grayscaleThumbnail.height * sizeof(unsigned char));

    PatchUtilities::normalizeContrast<unsigned char>(reinterpret_cast<unsigned char*>(network.input(0).data()), inputImageSize, 0.02f);
    network.apply();
}

void RobotMiddleInfo::applyColorNetwork(ECImage& ecImage)
{
    ASSERT(networkParameters.inputChannels == 3);
    fillGrayscaleThumbnail(ecImage);
    fillChromaThumbnails(ecImage);

    PixelTypes::GrayscaledPixel* yPos = grayscaleThumbnail[0];
    PixelTypes::GrayscaledPixel* uPos = redChromaThumbnail[0];
    PixelTypes::GrayscaledPixel* vPos = blueChromaThumbnail[0];
    PixelTypes::GrayscaledPixel* inputPos;

    inputPos = reinterpret_cast<PixelTypes::GrayscaledPixel*>(network.input(0).data());

    for(unsigned int pos = 0; pos < networkParameters.inputWidth * networkParameters.inputHeight; ++pos, ++yPos, ++uPos, ++vPos, inputPos += 3) {
        inputPos[0] = yPos[0];
        inputPos[1] = uPos[0];
        inputPos[2] = vPos[0];
    }

    network.apply();
}

void RobotMiddleInfo::boundingBoxes(LabelImage& labelImage, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary) {
    const float objectThreshold = LOGIT(objectThres);
    for(unsigned y = 0; y < networkParameters.outputHeight; ++y) {
        for(unsigned x = 0; x < networkParameters.outputWidth; ++x) {
            for(unsigned b = 0; b < networkParameters.outputAnchors; ++b) {
                // Check confidence objectThreshold for every Anchor, calculate bounding box only if confidence is above objectThreshold
                const size_t offset = (y * networkParameters.outputWidth + x)
                                    * networkParameters.outputAnchors
                                    * networkParameters.paramsPerAnchor
                                    + b * networkParameters.paramsPerAnchor;
                if(network.output(0)[offset] > objectThreshold) {
                    Eigen::Map<Eigen::Matrix<float, 5, 1>> pred(network.output(0).data() + offset);
                    LabelImage::Annotation box = predictionToBoundingBox(pred, y, x, b, cameraInfo);
                    if(static_cast<float>(fieldBoundary.getBoundaryY(static_cast<int>((box.lowerRight.x() + box.upperLeft.x()) / 2.f))) > box.lowerRight.y()) {
                        continue;
                    }
                    labelImage.annotations.emplace_back(box);
                }
            }
        }
    }
}

LabelImage::Annotation RobotMiddleInfo::predictionToBoundingBox(Eigen::Map<Eigen::Matrix<float, 5, 1>>& pred, unsigned int y, unsigned int x, unsigned int b, const CameraInfo& cameraInfo) const {
    pred.array() = 1.f / (1.f + (pred * -1).array().exp());
    ASSERT(pred(networkParameters.confidenceIndex) >= 0.f && pred(networkParameters.confidenceIndex) <= 1.f);
    ASSERT(pred(networkParameters.yMidIndex) >= 0.f && pred(networkParameters.yMidIndex) <= 1.f);
    ASSERT(pred(networkParameters.xMidIndex) >= 0.f && pred(networkParameters.xMidIndex) <= 1.f);
    ASSERT(pred(networkParameters.widthIndex) >= 0.f && pred(networkParameters.widthIndex) <= 1.f);
    ASSERT(pred(networkParameters.heightIndex) >= 0.f && pred(networkParameters.heightIndex) <= 1.f);

    // Bounding Box Position
    pred(networkParameters.yMidIndex) = (static_cast<float>(y) + pred(networkParameters.yMidIndex)) /
                                        static_cast<float>(networkParameters.outputHeight) *
                                        static_cast<float>(cameraInfo.height);
    pred(networkParameters.xMidIndex) = (static_cast<float>(x) + pred(networkParameters.xMidIndex)) /
                                        static_cast<float>(networkParameters.outputWidth) *
                                        static_cast<float>(cameraInfo.width);

    // Bounding Box Size
    // 0.5 = no change in size, above and below will scale exponentially
    pred(networkParameters.heightIndex) = networkParameters.anchors[b].y()
                                            * std::pow(networkParameters.sizeConversionFactor, 2.f * pred(3) - 1.f)
                                            * static_cast<float>(cameraInfo.height);
    pred(networkParameters.widthIndex) = networkParameters.anchors[b].x()
                                        * std::pow(networkParameters.sizeConversionFactor, 2.f * pred(4) - 1.f)
                                        * static_cast<float>(cameraInfo.width);

    LabelImage::Annotation box;
    box.upperLeft = Vector2f(pred(networkParameters.xMidIndex) - pred(networkParameters.widthIndex) / 2.f,
                            pred(networkParameters.yMidIndex) - pred(networkParameters.heightIndex) / 2.f);
    box.lowerRight = Vector2f(pred(networkParameters.xMidIndex) + pred(networkParameters.widthIndex) / 2.f,
                                pred(networkParameters.yMidIndex) + pred(networkParameters.heightIndex) / 2.f);
    box.confidence = pred(networkParameters.confidenceIndex);

    if(networkParameters.predictFallen) {
        box.fallen = pred(networkParameters.fallenClassIndex) > fallenThres;
    }
    else {
        box.fallen = false;
    }

    box.distance = -1.f;

    return box;
}

void RobotMiddleInfo::mergeObstacles(RobotObstaclesField& obstaclesField, RobotObstaclesIncomplete& obstaclesData, std::vector<RobotObstaclesImage::Obstacle>& obstacles, const VisionInfoIn* info_in, const CameraInfo& cameraInfo, ECImage& ecImage, RobotObstaclesIncomplete& otherObstaclesData)
{
    bool mergeObstacles = false;
    //const Vector2f& robotRotationDeviation = theMotionInfo.executedPhase == MotionPhase::stand ? pRobotRotationDeviationInStand : pRobotRotationDeviation;
    const Vector2f& robotRotationDeviation = pRobotRotationDeviation;
    do {
        //mergeObstacles = mergeLowerObstacles && cameraInfo.camera == CameraInfo::lower && !mergeObstacles && obstacles.size() >= 2;
        auto it = obstacles.begin();
        while(it != obstacles.end()) {
            RobotObstaclesImage::Obstacle& obstacleInImage = *it;
            RobotObstaclesField::Obstacle obstacleOnField;
            Matrix2f leftCovariance, rightCovariance;
            obstacleOnField.left = info_in->kinematicPose.imageToRobotXY(Vector2i(obstacleInImage.left, obstacleInImage.bottom).cast<float>(), cameraInfo);
            obstacleOnField.right = info_in->kinematicPose.imageToRobotXY(Vector2i(obstacleInImage.right, obstacleInImage.bottom).cast<float>(), cameraInfo);
            bool validObstacle =
                SpatialUtilities::possiblyOnFieldRRXY(obstacleOnField.left) &&
                SpatialUtilities::possiblyOnFieldRRXY(obstacleOnField.right);

            if(validObstacle) {
                obstacleOnField.fallen = obstacleInImage.fallen;
                obstacleOnField.type = RobotObstaclesField::Type::unknown;
                obstacleOnField.center = (obstacleOnField.left + obstacleOnField.right) * 0.5f;

                const Rangea range(obstacleOnField.right.angle(), obstacleOnField.left.angle());

                if(static_cast<float>(obstacleInImage.bottom) > static_cast<float>(ecImage.grayscaled.height) * 0.9f) {
                    for(const RobotObstaclesField::Obstacle& incompleteObstacle : otherObstaclesData.incompleteObstacles)
                    {
                        // const Pose2f inverseOdometryOffset = theOdometryData.inverse() * theOtherOdometryData;
                        const Rangea rangeLower((incompleteObstacle.right).angle(), (incompleteObstacle.left).angle());
                        if(range.min <= rangeLower.max && rangeLower.min <= range.max) {
                            obstacleInImage.bottomFound = false;
                        }
                    }
                }

                if(!trimObstacles || (validObstacle = trimObstacle(false, obstacleInImage, cameraInfo, ecImage) &&
                                                        SpatialUtilities::possiblyOnFieldRRXY(obstacleOnField.left) &&
                                                        SpatialUtilities::possiblyOnFieldRRXY(obstacleOnField.right) &&
                                                        (obstacleOnField.right - obstacleOnField.left).squaredNorm() <= SQUARE(750.f)))
                {
                    if(!obstacleInImage.bottomFound) {
                        JerseyClassifier::detectJersey(obstacleInImage, obstacleOnField, blackboard, info_in, cameraInfo, ecImage);
                        const_cast<RobotObstaclesIncomplete&>(obstaclesData).incompleteObstacles.emplace_back(obstacleOnField);
                    }
                    else if((validObstacle = obstacleInImage.right - obstacleInImage.left < static_cast<int>(ecImage.grayscaled.width / 2))) {
                        obstacleOnField.center = (obstacleOnField.left + obstacleOnField.right) * 0.5f;
                        JerseyClassifier::detectJersey(obstacleInImage, obstacleOnField, blackboard, info_in, cameraInfo, ecImage);
                        obstaclesField.obstacles.emplace_back(obstacleOnField);
                    }
                }
            }
            it = validObstacle ? it + 1 : obstacles.erase(it);
        }

        if(mergeObstacles) {
            bool mergedSomething;
            do {
                std::vector<bool> merged(obstacles.size(), false);
                std::vector<RobotObstaclesImage::Obstacle> mergedObstacles;
                mergedSomething = false;
                for(size_t i = 0; i < obstacles.size(); ++i)
                {
                    if(merged[i]) continue;
                    RobotObstaclesImage::Obstacle& a = obstacles[i];
                    const Rangei obstacleRange = Rangei(a.left, a.right);

                    for(size_t j = i + 1; j < obstacles.size(); ++j) {
                        RobotObstaclesImage::Obstacle& b = obstacles[j];
                        const Rangei otherObstacleRange = Rangei(b.left, b.right);
                        if(std::min(obstacleRange.max, otherObstacleRange.max) - std::max(obstacleRange.min, otherObstacleRange.min) > 0) {
                            a.top = std::min(a.top, b.top);
                            a.bottom = std::max(a.bottom, b.bottom);
                            a.left = std::min(a.left, b.left);
                            a.right = std::max(a.right, b.right);
                            mergedObstacles.emplace_back(a);
                            merged[i] = true;
                            merged[j] = true;
                            mergedSomething = true;
                        }
                    }
                    if(!merged[i]) mergedObstacles.emplace_back(a);
                }
                obstacles = mergedObstacles;
            }
            while(mergedSomething);
        }
    }
    while(mergeObstacles);
}

bool RobotMiddleInfo::trimObstacle(bool trimHeight, RobotObstaclesImage::Obstacle& obstacleInImage, const CameraInfo& cameraInfo, ECImage& ecImage) {
    const int minX = std::max(obstacleInImage.left, 0);
    const int minY = std::max(static_cast<int>(obstacleInImage.top + (obstacleInImage.bottom - obstacleInImage.top) / 2), 0);
    const int maxX = std::min(obstacleInImage.right, static_cast<int>(ecImage.grayscaled.width));
    const int maxY = std::min(obstacleInImage.bottom, static_cast<int>(ecImage.grayscaled.height));

    int stepSize = static_cast<int>(xyStep);
    std::vector<std::vector<int>> limits = {{}, {}};
    for(int y = minY; y < maxY; y += stepSize) {
        int offset = minX, step = static_cast<int>(xyStep);
        for(int side = 0; side < 2; ++side, offset = maxX - 1, step *= -1) {
            if((side == 0 && minX <= 10) || (side == 1 && maxX >= static_cast<int>(ecImage.grayscaled.width - 10))) {
                continue;
            }

            const PixelTypes::GrayscaledPixel* secondSpot = ecImage.grayscaled[y] + offset;
            short firstSpot = *secondSpot;
            std::function<bool(int)> comp = (side == 0) ? static_cast<std::function<bool(int)>>([maxX, stepSize](int a) -> bool {return a < maxX - stepSize;})
                                                        : static_cast<std::function<bool(int)>>([minX, stepSize](int a) -> bool {return a >= minX + stepSize;});
            for(int x = offset; comp(x); x += step, firstSpot = *secondSpot) {
                secondSpot += step;
                if(std::abs(*secondSpot - firstSpot) > minContrastDiff / 2) {
                    limits[side].emplace_back(x - step / 2);
                    break;
                }
            }
        }
    }
    std::sort(limits[0].begin(), limits[0].end());
    if(!limits[0].empty()) obstacleInImage.left = limits[0][limits[0].size() / 2];
    std::sort(limits[1].begin(), limits[1].end());
    if(!limits[1].empty()) obstacleInImage.right = limits[1][limits[1].size() / 2];

    if(trimHeight) {
        const Rangei yRange = Rangei(minY + (maxY - minY) / 4, maxY - 1);
        int step = stepSize, upperY = -1, lowerY = -1, botCan = obstacleInImage.bottom;
        for(int side = 0; side < 2; ++side, step *= -1) {
            bool satRow = false, nonSatRow = false;
            for(int y = (minY + maxY) / 2; yRange.isInside(y) && (!nonSatRow || !satRow); y += step) {
                const PixelTypes::GrayscaledPixel* spot = ecImage.saturated[y] + obstacleInImage.left;
                int x = obstacleInImage.left;
                for(; x < obstacleInImage.right && *spot > satThreshold; x += stepSize, spot += stepSize);

                if(x >= obstacleInImage.right) {
                    satRow = true;
                }
                else {
                    botCan = y;
                    nonSatRow = true;
                }
                if(satRow && nonSatRow)(side == 0 ? lowerY : upperY) = botCan;
            }
        }
        obstacleInImage.bottom = upperY != -1 ? upperY : (lowerY != -1 ? lowerY : obstacleInImage.bottom);
    }
    // TODO
    // Vector2f relativePosition;
    // Geometry::Circle ball;
    // const float radius = Transformation::imageToRobotHorizontalPlane(Vector2f((obstacleInImage.right + obstacleInImage.left) / 2, (obstacleInImage.top + obstacleInImage.bottom) / 2), theBallSpecification.radius, theCameraMatrix, theCameraInfo, relativePosition)
    //                     && Projection::calculateBallInImage(relativePosition, theCameraMatrix, theCameraInfo, theBallSpecification.radius, ball) ? ball.radius : -1.f;
    // const int minWidthInImage = std::max(minPixel, static_cast<int>(radius));
    const int minWidthInImage = minPixel;
    return obstacleInImage.right - obstacleInImage.left >= minWidthInImage && obstacleInImage.bottom - obstacleInImage.top >= minWidthInImage &&
            static_cast<float>(obstacleInImage.right - obstacleInImage.left) / static_cast<float>(maxX - minX) >= minBeforeAfterTrimRatio;
}

void RobotMiddleInfo::scanImage(std::vector<std::vector<Region>>& regions, const CameraInfo& cameraInfo, FieldBoundary& fieldBoundary, ECImage& ecImage) {
    std::vector<std::pair<int, int>> yLimits(ecImage.grayscaled.width / xyStep);
    for(unsigned int x = 0, index = 0; index < ecImage.grayscaled.width / xyStep; x += xyStep, ++index) {
        yLimits[index] = std::make_pair(fieldBoundary.getBoundaryY(static_cast<int>(x)), cameraInfo.height);
    }
    const float yRegionsDivisor = static_cast<float>(ecImage.grayscaled.height) / static_cast<float>(xyRegions);
    const float xRegionsDivisor = static_cast<float>(ecImage.grayscaled.width) / static_cast<float>(xyRegions);

    std::pair<const PixelTypes::GrayscaledPixel*, const PixelTypes::GrayscaledPixel*> upperRow = {ecImage.grayscaled[0], ecImage.saturated[0]};
    std::pair<const PixelTypes::GrayscaledPixel*, const PixelTypes::GrayscaledPixel*> midRow = upperRow;
    std::pair<const PixelTypes::GrayscaledPixel*, const PixelTypes::GrayscaledPixel*> lowerRow;
    for(unsigned int y = 0; y < ecImage.grayscaled.height - xyStep; y += xyStep, upperRow = midRow, midRow = lowerRow) {
        lowerRow = {ecImage.grayscaled[y + xyStep], ecImage.saturated[y + xyStep]};
        short leftLum = *midRow.first, leftSat = *midRow.second, midLum = leftLum, midSat = leftSat;
        const PixelTypes::GrayscaledPixel* rightLum = midRow.first;
        const PixelTypes::GrayscaledPixel* rightSat = midRow.second;

        for(unsigned int x = 0, index = 0; x < ecImage.grayscaled.width - xyStep; x += xyStep, ++index, leftLum = midLum, midLum = *rightLum, leftSat = midSat, midSat = *rightSat) {
            rightLum += xyStep;
            rightSat += xyStep;
            if(yLimits[index].first < static_cast<int>(y - xyStep) && yLimits[index].second > static_cast<int>(y + xyStep)) {
                bool horizontalChange = (leftSat < satThreshold || *rightSat < satThreshold) && std::abs(*rightLum - leftLum) > minContrastDiff;
                bool verticalChange = (*(upperRow.second + x) < satThreshold || *(lowerRow.second + x) < satThreshold) &&
                                    std::abs(static_cast<short>(*(upperRow.first + x)) - static_cast<short>(*(lowerRow.first + x))) > minContrastDiff;
                Classification spotClassification = horizontalChange ? (verticalChange ? Nothing : Horizontal) : (verticalChange ? Vertical : Nothing);
                if(spotClassification != Nothing) {
                    Region& region = regions[static_cast<int>(static_cast<float>(y) / yRegionsDivisor)][static_cast<int>(static_cast<float>(x) / xRegionsDivisor)];
                    ++region.contrastChanges[spotClassification];
                    if(static_cast<int>(y) > region.maxY)
                        region.maxY = static_cast<int>(y);
                }
                if(midSat < satThreshold && midLum >= brightnessThreshold) {
                    ++regions[static_cast<int>(static_cast<float>(y) / yRegionsDivisor)][static_cast<int>(static_cast<float>(x) / xRegionsDivisor)].brightSpots;
                }
            }
        }
    }
}


void RobotMiddleInfo::classifyRegions(std::vector<std::vector<Region>>& regions, ECImage& ecImage) {
    const int minSpotsToClassify = std::max(5, std::min(30, static_cast<int>((ecImage.grayscaled.width * ecImage.grayscaled.height) / (xyRegions * xyRegions) / (xyStep * xyStep) / 10)));

    for(unsigned int yRegionIndex = 0; yRegionIndex < xyRegions; ++yRegionIndex) {
        for(unsigned int xRegionIndex = 0; xRegionIndex < xyRegions; ++xRegionIndex) {
            Region& region = regions[yRegionIndex][xRegionIndex];
            region.regionIndices = Vector2i(xRegionIndex, yRegionIndex);

            const float hor = static_cast<float>(region.contrastChanges[0]);
            const float ver = static_cast<float>(region.contrastChanges[1]);
            if(hor + ver >= static_cast<float>(minSpotsToClassify)) {
                region.classification = (hor > ver ? ver / (ver + hor) : hor / (ver + hor)) > mixedThresh ? Mixed : (hor > ver ? Horizontal : Vertical);
            }
            region.bright = region.brightSpots >= minSpotsToClassify;
        }
    }
}


void RobotMiddleInfo::discardHomogeneousAreas(std::vector<std::vector<Region>>& regions) {
    std::vector<Vector2i> toDiscard;
    std::vector<std::vector<bool>> visited(xyRegions, std::vector<bool>(xyRegions, false));

    auto checkHomogeneity = [this](std::vector<std::vector<Region>>& regions, Vector2i& indices, Classification relevantClass) -> bool {
        int hetCounter = 0;
        for(int y = std::max(0, indices.y() - 1); hetCounter < minHetSpots && y <= std::min(static_cast<int>(xyRegions - 1), indices.y() + 1); ++y) {
            for(int x = std::max(0, indices.x() - 1); hetCounter < minHetSpots && x <= std::min(static_cast<int>(xyRegions - 1), indices.x() + 1); ++x) {
                const Region& neighbor = regions[y][x];
                if(neighbor.classification != Nothing && neighbor.classification != relevantClass) {
                    ++hetCounter;
                }
                if(hetCounter >= minHetSpots) {
                    return false;
                }
            }
        }
        return true;
    };

    for(unsigned int yRegionIndex = 0; yRegionIndex < xyRegions; ++yRegionIndex) {
        for(unsigned int xRegionIndex = 0; xRegionIndex < xyRegions; ++xRegionIndex) {
            Region& region = regions[yRegionIndex][xRegionIndex];
            if(region.classification == Nothing || region.classification == Mixed || visited[yRegionIndex][xRegionIndex]) {
                continue;
            }

            std::vector<Vector2i> toCheck = {Vector2i(xRegionIndex, yRegionIndex)};
            for(size_t index = 0; index < toCheck.size(); ++index) {
                Vector2i& indices = toCheck[index];
                if(checkHomogeneity(regions, indices, region.classification)) {
                    toDiscard.emplace_back(indices);
                    visited[indices.y()][indices.x()] = true;
                    std::vector<Vector2i> neighbors;
                    regionQuery(regions, indices, 1, neighbors, region.classification);
                    for(Vector2i& neighborIndices : neighbors) {
                        if(!visited[neighborIndices.y()][neighborIndices.x()]) {
                            visited[neighborIndices.y()][neighborIndices.x()] = true;
                            toCheck.emplace_back(neighborIndices);
                            toDiscard.emplace_back(neighborIndices);
                        }
                    }
                }
            }
        }
    }

    for(Vector2i& discardIndices : toDiscard) {
        regions[discardIndices.y()][discardIndices.x()].classification = Nothing;
    }

    for(unsigned int yRegionIndex = 0; yRegionIndex < xyRegions; ++yRegionIndex) {
        for(unsigned int xRegionIndex = 0; xRegionIndex < xyRegions; ++xRegionIndex) {
            Region& region = regions[yRegionIndex][xRegionIndex];
            if(region.classification == Nothing && region.bright && !checkHomogeneity(regions, region.regionIndices, Unknown)) {
                region.classification = Unknown;
            }
        }
    }
}

void RobotMiddleInfo::dbScan(std::vector<std::vector<Region>>& regions, std::vector<RobotObstaclesImage::Obstacle>& obstacles, const CameraInfo& cameraInfo, ECImage& ecImage) {
    for(std::vector<Region>& horizontalRegionLine : regions) {
        for(Region& region : horizontalRegionLine)  {
            if(region.classification == Nothing || region.clustered) {
                continue;
            }
            std::vector<Vector2i> neighbors;
            regionQuery(regions, region.regionIndices, 1, neighbors);
            if(neighbors.size() >= minNeighborPoints) {
                std::vector<Vector2i> cluster;
                Vector2i topLeft = region.regionIndices, bottomRight = Vector2i(region.regionIndices.x(), region.maxY);
                if(expandCluster(regions, region, neighbors, cluster, topLeft, bottomRight)) {
                    RobotObstaclesImage::Obstacle obstacle;
                    obstacle.top = static_cast<int>(static_cast<float>(topLeft.y()) * static_cast<float>(ecImage.grayscaled.height) / static_cast<float>(xyRegions));
                    obstacle.bottom = bottomRight.y();
                    obstacle.left = static_cast<int>(static_cast<float>(topLeft.x()) * static_cast<float>(ecImage.grayscaled.width) / static_cast<float>(xyRegions));
                    obstacle.right = static_cast<int>(static_cast<float>(bottomRight.x() + 1) * static_cast<float>(ecImage.grayscaled.width) / static_cast<float>(xyRegions));
                    obstacle.bottomFound = true;
                    obstacle.fallen = false;

                    if(!trimObstacles || trimObstacle(true, obstacle, cameraInfo, ecImage)) {
                        obstacles.emplace_back(obstacle);
                    }
                }
            }
        }
    }
}

bool RobotMiddleInfo::expandCluster(std::vector<std::vector<Region>>& regions, Region& region, std::vector<Vector2i>& neighbors, std::vector<Vector2i>& cluster, Vector2i& topLeft, Vector2i& bottomRight) {
    bool containsMixedRegions = false;
    cluster.emplace_back(region.regionIndices);
    region.clustered = true;
    containsMixedRegions = containsMixedRegions || region.classification == Mixed;

    for(unsigned int i = 0; i < neighbors.size(); ++i) {
        Vector2i& neighborIndices = neighbors[i];
        Region& currentRegion = regions[neighborIndices.y()][neighborIndices.x()];
        if(currentRegion.clustered) {
            continue;
        }

        cluster.emplace_back(neighborIndices);
        containsMixedRegions = containsMixedRegions || currentRegion.classification == Mixed;
        currentRegion.clustered = true;
        if(currentRegion.classification != Unknown) {
            topLeft = Vector2i(std::min(currentRegion.regionIndices.x(), topLeft.x()), std::min(currentRegion.regionIndices.y(), topLeft.y()));
            bottomRight = Vector2i(std::max(currentRegion.regionIndices.x(), bottomRight.x()), std::max(currentRegion.maxY, bottomRight.y()));
        }

        std::vector<Vector2i> nextNeighbors;
        regionQuery(regions, neighborIndices, 1, nextNeighbors);
        if(nextNeighbors.size() >= minNeighborPoints) {
            for(Vector2i& nextNeighbor : nextNeighbors) {
                if(!regions[nextNeighbor.y()][nextNeighbor.x()].clustered) {
                    neighbors.emplace_back(nextNeighbor);
                }
            }
        }
    }

    for(int yRegionIndex = topLeft.y() - 1; yRegionIndex >= 0 && yRegionIndex == topLeft.y() - 1; --yRegionIndex) {
        for(int xRegionIndex = topLeft.x(); xRegionIndex <= bottomRight.x(); ++xRegionIndex) {
            if(regions[yRegionIndex][xRegionIndex].bright) {
                cluster.emplace_back(xRegionIndex, yRegionIndex);
                topLeft = Vector2i(topLeft.x(), yRegionIndex);
            }
        }
    }
    
    return containsMixedRegions;
}

void RobotMiddleInfo::regionQuery(std::vector<std::vector<Region>>& regions, Vector2i& regionIndices, int dis, std::vector<Vector2i>& neighbors, Classification classificationToSearch) {
    for(int y = std::max(regionIndices.y() - dis, 0); y < std::min(regionIndices.y() + dis + 1, static_cast<int>(xyRegions)); ++y) {
        for(int x = std::max(regionIndices.x() - dis, 0); x < std::min(regionIndices.x() + dis + 1, static_cast<int>(xyRegions)); ++x) {
            if((classificationToSearch == Default && regions[y][x].classification != Nothing) || (classificationToSearch == regions[y][x].classification)) {
                neighbors.emplace_back(x, y);
            }
        }
    }
}