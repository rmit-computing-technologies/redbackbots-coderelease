/**
 * @file BallDetector.cpp
 *
 * This file implements a module that detects balls in images with a neural network.
 *
 * @author Bernd Poppinga
 * @author Felix Thielke
 * @author Gerrit Felsch
 * @author RedbackBots
 */

#include "perception/vision/detector/BallDetector.hpp"
#include "perception/vision/VisionInfoMiddle.hpp"

#include "utils/debug/Assert.hpp"
#include "utils/SpatialUtilities.hpp"
#include "perception/kinematics/RobotPose.hpp"

#include <asmjit/asmjit.h>

BallDetector::BallDetector(Blackboard* blackboard, asmjit::JitRuntime* jitRuntime):
    Detector("BallDetector"),
    encoder(jitRuntime),
    classifier(jitRuntime),
    corrector(jitRuntime),
    encModel(nullptr),
    clModel(nullptr),
    corModel(nullptr)
{
    configure(blackboard);

    std::string modelDir = blackboard->config["vision.ml.modeldir"].as<std::string>();
    std::string encoderFile = modelDir + "/BallDetector/encoder.h5";
    std::string classifierFile = modelDir + "/BallDetector/classify.h5";
    std::string correctorFile  = modelDir + "/BallDetector/corrector.h5";

    llog(INFO) << NDEBUG_LOGSYMB << "Ball Detector Loading encoder Model from file " << encoderFile << std::endl;
    llog(INFO) << NDEBUG_LOGSYMB << "Ball Detector Loading classifier Model from file " << classifierFile << std::endl;
    llog(INFO) << NDEBUG_LOGSYMB << "Ball Detector Loading corrector Model from file " << correctorFile << std::endl;

    encModel = std::make_unique<NeuralNetwork::Model>(encoderFile);
    encModel->setInputUInt8(0);
    clModel = std::make_unique<NeuralNetwork::Model>(classifierFile);
    corModel = std::make_unique<NeuralNetwork::Model>(correctorFile);

    encoder.compile(*encModel);
    classifier.compile(*clModel);
    corrector.compile(*corModel);

    ASSERT(encoder.numOfInputs() == 1);
    ASSERT(classifier.numOfInputs() == 1);
    ASSERT(corrector.numOfInputs() == 1);

    ASSERT(classifier.numOfOutputs() == 1);
    ASSERT(corrector.numOfOutputs() == 1);
    ASSERT(encoder.numOfOutputs() == 1);

    ASSERT(encoder.input(0).rank() == 3);
    ASSERT(encoder.input(0).dims(0) == encoder.input(0).dims(1));
    ASSERT(encoder.input(0).dims(2) == 1);

    ASSERT(classifier.output(0).rank() == 1);
    ASSERT(classifier.output(0).dims(0) == 1 && corrector.output(0).dims(0) == 3);
    patchSize = encoder.input(0).dims(0);

    llog(INFO) << NDEBUG_LOGSYMB << "Ball detector Models loaded and compiled" << std::endl;
}

BallDetector::~BallDetector() {
    encModel = nullptr;
    clModel = nullptr;
    corModel = nullptr;
}

void BallDetector::configure(Blackboard* blackboard) {

}

void BallDetector::resetMiddleInfo(VisionInfoMiddle* info_middle) {

}

void BallDetector::resetVisionOut(VisionInfoOut* info_out) {
    // for (int i = 0; i < info_out->balls.size(); ++i) {
    //     info_out->balls[i].reset();
    // }
    info_out->balls.clear();
}

void BallDetector::detect_(const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << __PRETTY_FUNCTION__ << std::endl;
    // llog(INFO) << NDEBUG_LOGSYMB << "tick BallDetector" << std::endl;

    detect_(CameraInfo::Camera::top, info_in, info_middle, info_out);

    detect_(CameraInfo::Camera::bot, info_in, info_middle, info_out);
}


void BallDetector::detect_(CameraInfo::Camera whichCamera, const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {  
    const CameraInfo& cameraInfo = info_in->cameraInfo[whichCamera];
    const CameraImage* cameraImage = info_in->image[whichCamera];
	ECImage& ecImage = info_middle->ecImage[whichCamera];
    BallSpots& ballSpots = info_middle->ballSpots[whichCamera];
    PreviousBalls& previousBalls = info_middle->previousBalls[whichCamera];

    std::vector<BallInfo>& ballInfo = info_out->balls;

    // preset top camera depending on which camera we are detecting on and set status to default (false)
    const bool isTopCamera = whichCamera == CameraInfo::Camera::top;

    if (!encoder.valid() || !classifier.valid() || !corrector.valid()) {
        return;
    }
    
    if (ballSpots.ballSpots.empty()) {
        return;
    }

    float prob = guessedThreshold;
    Vector2f ballPosition;
    float radius;
    bool first = true;
    for(std::size_t i = 0; i < ballSpots.ballSpots.size(); ++i) {
        prob = apply(ballSpots.ballSpots[i], ballPosition, radius, info_in, cameraInfo, ecImage);
        // llog(INFO) << NDEBUG_LOGSYMB << "Camera: " << whichCamera << std::endl;
        // llog(INFO) << NDEBUG_LOGSYMB << "Detecting spot " << i <<  " with confidence " << prob * 100 << "%"<< std::endl;

        if (prob > guessedThreshold) {
            if (first) {
                previousBalls.reset();
                previousBalls.timestamp = cameraImage->timestamp;
                first = false;
            }
            // llog(INFO) << NDEBUG_LOGSYMB << "Detecting spot " << i <<  " with confidence " << prob * 100 << "%"<< std::endl;
            // llog(INFO) << NDEBUG_LOGSYMB << "Ball position: " << ballPosition.x() << ","<< ballPosition.y() << std::endl;

            // Account for robot leaning, if necessary
            float diff = 190*std::tan(info_in->latestAngleX);
            ballPosition.y() -= diff;

            BallInfo newBall;
            newBall.status = prob >= acceptThreshold ? BallInfo::seen : BallInfo::guessed;
            newBall.radius = radius;
            newBall.imageCoords = ballPosition.cast<int>();
            newBall.topCamera = isTopCamera;
            newBall.rr = info_in->kinematicPose.imageToRobotRelative(newBall.imageCoords, cameraInfo, BALL_RADIUS);

            // Test if we need this
            //float robot_height = 500;
            //float error = 30 * newBall.rr.distance() / robot_height;
            //newBall.rr.distance() -= error;

            // llog(INFO) << NDEBUG_LOGSYMB << "Ball status: " << newBall.status << std::endl;

            ballInfo.emplace_back(newBall);

            // update previous balls for ball spot calculations
            previousBalls.balls.emplace_back(newBall.imageCoords);
        }
    }

    // TODO - (MP) Need game state for special cases
    // Special ball handling for penalty goal keeper
    // if ((theGameState.state == GameState::opponentPenaltyShot || theGameState.state == GameState::opponentPenaltyKick) &&
    //     theMotionInfo.isKeyframeMotion(KeyframeMotionRequest::sitDownKeeper)) {

    //     Vector2f inImageLowPoint;
    //     Vector2f inImageUpPoint;
    //     if (Transformation::robotToImage(theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOwnGoalArea + 350.f, 0.f), theCameraMatrix, theCameraInfo, inImageLowPoint)
    //     && Transformation::robotToImage(theRobotPose.inverse() * Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0.f), theCameraMatrix, theCameraInfo, inImageUpPoint))
    //     {
    //         const int lowerY = std::min(static_cast<int>(inImageLowPoint.y()), theCameraInfo.height);
    //         const int upperY = std::max(static_cast<int>(inImageUpPoint.y()), 0);

    //         std::vector<Vector2i> sortedBallSpots = ballSpots.ballSpots;
    //         std::sort(sortedBallSpots.begin(), sortedBallSpots.end(), [&](const Vector2i& a, const Vector2i& b) {return a.y() > b.y(); });

    //         for (const Vector2i& spot : sortedBallSpots) {
    //             if (spot.y() < lowerY && spot.y() > upperY) {
    //                 theBallPercept.positionInImage = spot.cast<float>();

    //                 // TL: Function is called with hardcoded numbers here, as this parameter is only for backward compatibility to previous approach and should vanish hopefully soon.
    //                 if(theMeasurementCovariance.transformWithCovLegacy(theBallPercept.positionInImage, theBallSpecification.radius, Vector2f(0.04f, 0.06f),
    //                                                          theBallPercept.positionOnField, theBallPercept.covarianceOnField))
    //                 {
    //                     theBallPercept.status = BallInfo::seen;
    //                 }

    //                 theBallPercept.radiusInImage = 30.f;
    //             }
    //         }
    //     }
    // }
}

float BallDetector::apply(const Vector2i& ballSpot, Vector2f& ballPosition, float& predRadius, const VisionInfoIn* info_in, const CameraInfo& cameraInfo, ECImage& ecImage)
{
    Geometry::Circle ball;
    PointF relativePoint = info_in->kinematicPose.imageToRobotXY(ballSpot.cast<float>(), cameraInfo, BALL_RADIUS);

    if (!SpatialUtilities::possiblyOnFieldRRXY(relativePoint)) { //|| leftBallPointI < 0 || rightBallPointI < 0)
        return -1.f;
    }

    // Scuffed -> take diameter calculations from old BallDetector (MO)
    PointF leftBallPointRR = {relativePoint.x(), relativePoint.y() - BALL_RADIUS};
    int leftBallPointI = info_in->kinematicPose.robotToImageXY(leftBallPointRR, cameraInfo, BALL_RADIUS).x();
    PointF rightBallPointRR = {relativePoint.x(), relativePoint.y() + BALL_RADIUS};
    int rightBallPointI = info_in->kinematicPose.robotToImageXY(rightBallPointRR, cameraInfo, BALL_RADIUS).x();

    // llog(DEBUG) << NDEBUG_LOGSYMB << "leftBallPointI: " << info_in->kinematicPose.robotToImageXY(leftBallPointRR, cameraInfo, 0) << std::endl;
    // llog(DEBUG) << NDEBUG_LOGSYMB << "rightBallPointI: " << info_in->kinematicPose.robotToImageXY(rightBallPointRR, cameraInfo, BALL_RADIUS * 2) << std::endl;

    // if(!(RobotPose::imageToRobotHorizontalPlane(ballSpot.cast<float>(), BALL_RADIUS, cameraInfo, relativePoint)
    //    && Projection::calculateBallInImage(relativePoint, cameraInfo, BALL_RADIUS, ball))) 
    // {
    //     return -1.f;
    // }

    ball.center = relativePoint;
    //llog(DEBUG) << NDEBUG_LOGSYMB << "ball center: " << ball.center << std::endl;
    ball.radius = std::abs(leftBallPointI - rightBallPointI) / 2;
    //llog(DEBUG) << NDEBUG_LOGSYMB << "ball radius: " << ball.radius << std::endl;
    
    int ballArea = static_cast<int>(ball.radius * ballAreaFactor);
    ballArea += 4 - (ballArea % 4);

    if(useFloat) {
        PatchUtilities::extractPatch(ballSpot, Vector2i(ballArea, ballArea), Vector2i(patchSize, patchSize), ecImage.grayscaled, encoder.input(0).data(), extractionMode);
        switch(normalizationMode) 
        {
            case BallDetector::NormalizationMode::normalizeContrast:
                PatchUtilities::normalizeContrast(encoder.input(0).data(), Vector2i(patchSize, patchSize), normalizationOutlierRatio);
                break;
            case BallDetector::NormalizationMode::normalizeBrightness:
                PatchUtilities::normalizeBrightness(encoder.input(0).data(), Vector2i(patchSize, patchSize), normalizationOutlierRatio);
        }
    } else {
        PatchUtilities::extractPatch(ballSpot, Vector2i(ballArea, ballArea), Vector2i(patchSize, patchSize), ecImage.grayscaled, reinterpret_cast<unsigned char*>(encoder.input(0).data()), extractionMode);
        switch(normalizationMode)
        {
            case BallDetector::NormalizationMode::normalizeContrast:
                PatchUtilities::normalizeContrast(reinterpret_cast<unsigned char*>(encoder.input(0).data()), Vector2i(patchSize, patchSize), normalizationOutlierRatio);
                break;
            case BallDetector::NormalizationMode::normalizeBrightness:
                PatchUtilities::normalizeBrightness(reinterpret_cast<unsigned char*>(encoder.input(0).data()), Vector2i(patchSize, patchSize), normalizationOutlierRatio);
        }
    }

    const float stepSize = static_cast<float>(ballArea) / static_cast<float>(patchSize);

    // encode patch
    encoder.apply();

    // classify
    classifier.input(0) = encoder.output(0);
    classifier.apply();
    const float pred = classifier.output(0)[0];

    // predict ball position if poss for ball is high enough
    if(pred > guessedThreshold) {
        corrector.input(0) = encoder.output(0);
        corrector.apply();
        ballPosition.x() = (corrector.output(0)[0] - patchSize / 2) * stepSize + ballSpot.x();
        ballPosition.y() = (corrector.output(0)[1] - patchSize / 2) * stepSize + ballSpot.y();
        predRadius = corrector.output(0)[2] * stepSize;
    }

    return pred;
}