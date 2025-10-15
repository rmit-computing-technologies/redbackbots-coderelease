/**
 * @file RefereeGestureDetector.cpp
 *
 * This file implements a module that detects referee gestures.
 *
 * @author Thomas Röfer
 * @author RedbackBots
*/

#include "perception/vision/referee/RefereeGestureDetector.hpp"

#include "perception/vision/VisionInfoMiddle.hpp"
#include "perception/vision/VisionInfoOut.hpp"

#include <algorithm>

RefereeGestureDetector::RefereeGestureDetector(Blackboard* blackboard):
    Detector("RefereeGestureDetector")
{
    configure(blackboard);

    history.reserve(bufferSize);

    llog(INFO) << NDEBUG_LOGSYMB << "RefereeGestureDetector loaded" << std::endl;
}

RefereeGestureDetector::~RefereeGestureDetector() {
    blackboard = nullptr;
}

void RefereeGestureDetector::configure(Blackboard* blackboard) {
    this->blackboard = blackboard;
}

void RefereeGestureDetector::resetMiddleInfo(VisionInfoMiddle* info_middle) {

}

void RefereeGestureDetector::resetVisionOut(VisionInfoOut* info_out) {
    info_out->refereeGesture.gesture = RefereeGesture::none;

    if(readFrom(gameController, gameState) != STATE_STANDBY) {
        resetHistogram();
    }
}

void RefereeGestureDetector::detect_(const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << __PRETTY_FUNCTION__ << std::endl;

    detect_(CameraInfo::Camera::top, info_in, info_middle, info_out);

    // detect_(CameraInfo::Camera::bot, info_in, info_middle, info_out);
}

void RefereeGestureDetector::detect_(CameraInfo::Camera whichCamera, const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {  
    const CameraInfo& cameraInfo = info_in->cameraInfo[whichCamera];
    const CameraImage* cameraImage = info_in->image[whichCamera];

    RefereeKeypoints& refereeKeypoints = info_middle->refereeKeypoints;
    RefereeGesture& refereeGesture = info_out->refereeGesture;

    // Detect gesture and filter it over time.
    updateHistogram((!mustCrossMiddle || crossesMiddle(cameraImage, refereeKeypoints)) && pointsLowEnough(refereeKeypoints) ? detectGesture(refereeKeypoints) : RefereeGesture::Gesture::none);
    refereeGesture.gesture = getPredominantGesture();
}

bool RefereeGestureDetector::crossesMiddle(const CameraImage* cameraImage, RefereeKeypoints& keypoints) const {
    bool isLeft = false;
    bool isRight = false;
    for(int i = 0; i != RefereeKeypoints::Keypoint::count; i++) {
        if(keypoints.points[i].valid) {
            isLeft |= keypoints.points[i].position.x() < cameraImage->width;
            isRight |= keypoints.points[i].position.x() >= cameraImage->width;
        }
    }
    return isLeft && isRight;
}

bool RefereeGestureDetector::pointsLowEnough(RefereeKeypoints& keypoints) const {
  Vector2f dummy;
  for(int i = 0; i != RefereeKeypoints::Keypoint::count; i++) {
    const RefereeKeypoints::Point& point = keypoints.points[i];
    if(point.valid && point.position.y() > yThreshold) {
        return true;
    }
  }
  return false;
}

const RefereeKeypoints::Point& RefereeGestureDetector::getOrdered(const RefereeKeypoints::Keypoint keypoint, RefereeKeypoints& keypoints) const
{
  if(keypoint == RefereeKeypoints::nose) {
    return keypoints.points[RefereeKeypoints::nose];
  }
  else {
    const RefereeKeypoints::Keypoint mirror = static_cast<RefereeKeypoints::Keypoint>((keypoint - 1 ^ 1) + 1);
    return keypoints.points[(keypoints.points[keypoint].position.x() >= keypoints.points[mirror].position.x())
                               == (keypoint < mirror) ? keypoint : mirror];
  }
}

RefereeGesture::Gesture RefereeGestureDetector::detectGesture(RefereeKeypoints& keypoints) const {
    const Vector2f refereeOnField(0, (((FIELD_WIDTH / 2) + (FIELD_WIDTH / 2 + FIELD_WIDTH_OFFSET)) / 2.f) * (readFrom(gameController, leftTeam) ? 1 : -1));
    const Vector2f refereeOffset = refereeOnField - readFrom(stateEstimation, robotPos).convertToVector();
    const float yScale = refereeOffset.norm() / 3000.f;
    const float xScale = yScale / std::cos(std::abs(refereeOffset.angle()) - 90_deg);

    // Go through all rules.
    for(const Rule& rule : rules)
    {
        // Go through all constraints. If one is not satisfied, the gesture is rejected.
        for(const Constraint& constraint : rule.constraints) {
            const RefereeKeypoints::Point& from = getOrdered(constraint.from, keypoints);
            const RefereeKeypoints::Point& to = getOrdered(constraint.to, keypoints);
            if(!from.valid || !to.valid) {
                goto gestureRejected;
            }

            Vector2f diff(to.position - from.position);
            diff.x() *= xScale;
            diff.y() *= -yScale;

            if(!constraint.distance.isInside(diff.norm())) {
                goto gestureRejected;
            }

            const Angle angle = Angle::normalize(diff.angle() + M_PI_2);
            if(!constraint.direction.isInside(angle)) {
                goto gestureRejected;
            }
        }

        // A gesture was found.
        return rule.gesture;

    gestureRejected:
        continue;
    }
    
    return RefereeGesture::Gesture::none;
}

void RefereeGestureDetector::updateHistogram(const RefereeGesture::Gesture gesture) {
    if(history.size() == history.capacity()) {
        --histogram[history.back()];
    }
    history.push_front(gesture);
    ++histogram[gesture];
}

RefereeGesture::Gesture RefereeGestureDetector::getPredominantGesture() const
{
    std::array<int, RefereeGesture::Gesture::count> histogram = this->histogram;

    // If the wide fullTime gesture was detected, the substitution gesture is also fullTime.
    if(histogram[RefereeGesture::Gesture::fullTime]) {
        histogram[RefereeGesture::Gesture::fullTime] += histogram[RefereeGesture::Gesture::substitution];
    }

    // Determine most frequent gesture, ignoring "none".
    const auto predominantGesture = std::max_element(histogram.begin() + 1, histogram.end());

    // Return most frequent gesture if predominant enough, otherwise "none".
    return *predominantGesture < minDetectionRatio * history.capacity() ? RefereeGesture::Gesture::none
            : static_cast<RefereeGesture::Gesture>(predominantGesture - histogram.begin());
}