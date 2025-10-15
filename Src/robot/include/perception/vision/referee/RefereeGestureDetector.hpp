/**
 * @file RefereeGestureDetector.hpp
 *
 * This file declares a module that detects referee gestures.
 *
 * @author Thomas Röfer
 * @author RedbackBots
*/

#pragma once

#include "perception/vision/Detector.hpp"
#include "blackboard/Blackboard.hpp"

#include "types/math/Range.hpp"
#include "types/RingBuffer.h"
#include "utils/defs/FieldDefinitions.hpp"
// #include "Representations/Infrastructure/GameState.h"
#include "types/vision/RefereeKeypoints.hpp"
#include "types/vision/RefereeGesture.hpp"

class RefereeGestureDetector : public Detector {
public:
    RefereeGestureDetector(Blackboard* blackboard);
    virtual ~RefereeGestureDetector();

    // Resets middle info
    virtual void resetMiddleInfo(VisionInfoMiddle* info_middle);
    virtual void resetVisionOut(VisionInfoOut* info_out);

protected:
    virtual void detect_(const VisionInfoIn* info_in, 
        VisionInfoMiddle* info_middle,
        VisionInfoOut* info_out);

private:
    struct Constraint {
        RefereeKeypoints::Keypoint from; /**< The first keypoint the position of which is checked. */
        RefereeKeypoints::Keypoint to; /**< The second keypoint the position of which is checked. */
        Rangef distance; /**< The distance between the two keypoints must be in this range (in pixels). */
        Rangea direction; /**< The direction from \c from to \c to must be in this range (in radians). */
    };

    struct Rule{
        RefereeGesture::Gesture gesture; /**< The gesture detected by this rule. */
        std::vector<Constraint> constraints; /**< The constraints that must be satisfied to detect the gesture. */
    };

    Blackboard* blackboard = nullptr;

    unsigned bufferSize = 10; /**< Number of gesture detections buffered for majority vote. */
    unsigned maxSize = 30; /**< Maximum number of gesture detections allowed in a single hsitory. */
    float minDetectionRatio = 0.4; /**< Minimum ratio of buffered gestures that make up an accepted gesture. */
    bool mustCrossMiddle = false; /**< Must there be keypoints in both halves of the image to accept a gesture? */
    float yThreshold = 128; /**< At least one point must be below this threshold. */
    // TODO: initialise rules
    std::vector<Rule> rules = { /**< The rules made up of constraints that must be satisfied to detect a gesture. */
        {
            RefereeGesture::Gesture::standbyToReady,
            {
                { // \.
                    RefereeKeypoints::Keypoint::rightShoulder,
                    RefereeKeypoints::Keypoint::rightElbow,
                    {20, 110},
                    {150_deg, -90_deg}
                },
                { // \.
                    RefereeKeypoints::Keypoint::rightShoulder,
                    RefereeKeypoints::Keypoint::rightWrist,
                    {50, 220},
                    {100_deg, -110_deg}
                },
                { //  ./
                    RefereeKeypoints::Keypoint::leftShoulder,
                    RefereeKeypoints::Keypoint::leftElbow,
                    {20, 110},
                    {90_deg, -150_deg}
                },
                { //  ./
                    RefereeKeypoints::Keypoint::leftShoulder,
                    RefereeKeypoints::Keypoint::leftWrist,
                    {50, 220},
                    {110_deg, -100_deg}
                }
            }
        },
        {
            RefereeGesture::Gesture::kickInBlue,
            {
                { // -O
                    RefereeKeypoints::Keypoint::rightShoulder,
                    RefereeKeypoints::Keypoint::rightElbow,
                    {20, 110},
                    {-120_deg, -60_deg}
                },
                { // -O
                    RefereeKeypoints::Keypoint::rightShoulder,
                    RefereeKeypoints::Keypoint::rightWrist,
                    {50, 220},
                    {-110_deg, -70_deg}
                },
                { //  O|
                    RefereeKeypoints::Keypoint::leftShoulder,
                    RefereeKeypoints::Keypoint::leftElbow,
                    {20, 110},
                    {-30_deg, 35_deg}
                },
                { //  O|
                    RefereeKeypoints::Keypoint::leftElbow,
                    RefereeKeypoints::Keypoint::leftWrist,
                    {20, 110},
                    {-30_deg, 35_deg}
                },
                { //  O|
                    RefereeKeypoints::Keypoint::leftShoulder,
                    RefereeKeypoints::Keypoint::leftWrist,
                    {50, 220},
                    {-15_deg, 30_deg}
                }
            }
        },
        {
            RefereeGesture::Gesture::kickInRed,
            {
                { // |O
                    RefereeKeypoints::Keypoint::rightShoulder,
                    RefereeKeypoints::Keypoint::rightElbow,
                    {20, 110},
                    {-35_deg, 30_deg}
                },
                { // |O
                    RefereeKeypoints::Keypoint::rightElbow,
                    RefereeKeypoints::Keypoint::rightWrist,
                    {20, 110},
                    {-35_deg, 30_deg}
                },
                { // |O
                    RefereeKeypoints::Keypoint::rightShoulder,
                    RefereeKeypoints::Keypoint::rightWrist,
                    {20, 220},
                    {-30_deg, 15_deg}
                },
                { //  O-
                    RefereeKeypoints::Keypoint::leftShoulder,
                    RefereeKeypoints::Keypoint::leftElbow,
                    {20, 110},
                    {60_deg, 120_deg}
                },
                { //  O-
                    RefereeKeypoints::Keypoint::leftShoulder,
                    RefereeKeypoints::Keypoint::leftWrist,
                    {50, 220},
                    {70_deg, 110_deg}
                }
            }
        },
        {
            RefereeGesture::Gesture::goalKickBlue,
            {
                { // \O
                    RefereeKeypoints::Keypoint::rightShoulder,
                    RefereeKeypoints::Keypoint::rightElbow,
                    {20, 110},
                    {-180_deg, -90_deg}
                },
                { // \O
                    RefereeKeypoints::Keypoint::rightShoulder,
                    RefereeKeypoints::Keypoint::rightWrist,
                    {50, 220},
                    {-180_deg, -110_deg}
                },
                { //  O|
                    RefereeKeypoints::Keypoint::leftShoulder,
                    RefereeKeypoints::Keypoint::leftElbow,
                    {20, 110},
                    {-30_deg, 35_deg}
                },
                { //  O|
                    RefereeKeypoints::Keypoint::leftShoulder,
                    RefereeKeypoints::Keypoint::leftWrist,
                    {20, 220},
                    {-15_deg, 30_deg}
                }
            }
        },
        {
            RefereeGesture::Gesture::goalKickRed,
            {
                { // |O
                    RefereeKeypoints::Keypoint::rightShoulder,
                    RefereeKeypoints::Keypoint::rightElbow,
                    {20, 110},
                    {-35_deg, 30_deg}
                },
                { // |O
                    RefereeKeypoints::Keypoint::rightShoulder,
                    RefereeKeypoints::Keypoint::rightWrist,
                    {50, 220},
                    {-30_deg, 15_deg}
                },
                { //  O/
                    RefereeKeypoints::Keypoint::leftShoulder,
                    RefereeKeypoints::Keypoint::leftElbow,
                    {20, 110},
                    {90_deg, -180_deg}
                },
                { //  O/
                    RefereeKeypoints::Keypoint::leftShoulder,
                    RefereeKeypoints::Keypoint::leftWrist,
                    {50, 220},
                    {110_deg, -180_deg}
                }
            }
        },
        {
            RefereeGesture::Gesture::cornerKickBlue,
            {
                { // /O
                    RefereeKeypoints::Keypoint::rightShoulder,
                    RefereeKeypoints::Keypoint::rightElbow,
                    {20, 110},
                    {-90_deg, -5.00001_deg}
                },
                { // /O
                    RefereeKeypoints::Keypoint::rightShoulder,
                    RefereeKeypoints::Keypoint::rightWrist,
                    {50, 220},
                    {-70_deg, -30_deg}
                },
                { //  O|
                    RefereeKeypoints::Keypoint::leftShoulder,
                    RefereeKeypoints::Keypoint::leftElbow,
                    {20, 110},
                    {-30_deg, 35_deg}
                },
                { //  O|
                    RefereeKeypoints::Keypoint::leftShoulder,
                    RefereeKeypoints::Keypoint::leftWrist,
                    {50, 220},
                    {-15_deg, 30_deg}
                }
            }
        },
        {
            RefereeGesture::Gesture::cornerKickRed,
            {
                { // |O
                    RefereeKeypoints::Keypoint::rightShoulder,
                    RefereeKeypoints::Keypoint::rightElbow,
                    {20, 110},
                    {-35_deg, 30_deg}
                },
                { // |O
                    RefereeKeypoints::Keypoint::rightShoulder,
                    RefereeKeypoints::Keypoint::rightWrist,
                    {50, 220},
                    {-30_deg, 15_deg}
                },
                { //  O\                                  */
                    RefereeKeypoints::Keypoint::leftShoulder,
                    RefereeKeypoints::Keypoint::leftElbow,
                    {20, 110},
                    {5.00001_deg, 90_deg}
                },
                { //  O\                                  */
                    RefereeKeypoints::Keypoint::leftShoulder,
                    RefereeKeypoints::Keypoint::leftWrist,
                    {50, 220},
                    {30_deg, 70_deg}
                }
            }
        },
        {
            RefereeGesture::Gesture::pushingFreeKickBlue,
            {
                { // -O
                    RefereeKeypoints::Keypoint::rightShoulder,
                    RefereeKeypoints::Keypoint::rightElbow,
                    {20, 110},
                    {-120_deg, -60_deg}
                },
                { // -O
                    RefereeKeypoints::Keypoint::rightShoulder,
                    RefereeKeypoints::Keypoint::rightWrist,
                    {50, 220},
                    {-110_deg, -70_deg}
                },
                { //  O>
                    RefereeKeypoints::Keypoint::leftShoulder,
                    RefereeKeypoints::Keypoint::leftElbow,
                    {20, 110},
                    {0_deg, 70_deg}
                },
                { //  O>
                    RefereeKeypoints::Keypoint::leftElbow,
                    RefereeKeypoints::Keypoint::leftWrist,
                    {5, 110},
                    {-140_deg, -90_deg}
                }
            }
        },
        {
            RefereeGesture::Gesture::pushingFreeKickRed,
            {
                { // <O
                    RefereeKeypoints::Keypoint::rightShoulder,
                    RefereeKeypoints::Keypoint::rightElbow,
                    {20, 110},
                    {-70_deg, 0_deg}
                },
                { // <O
                    RefereeKeypoints::Keypoint::rightElbow,
                    RefereeKeypoints::Keypoint::rightWrist,
                    {5, 110},
                    {90_deg, 140_deg}
                },
                { // O-
                    RefereeKeypoints::Keypoint::leftShoulder,
                    RefereeKeypoints::Keypoint::leftElbow,
                    {20, 110},
                    {60_deg, 120_deg}
                },
                { // O-
                    RefereeKeypoints::Keypoint::leftShoulder,
                    RefereeKeypoints::Keypoint::leftWrist,
                    {50, 220},
                    {70_deg, 110_deg}
                }
            }
        },
        {
            RefereeGesture::Gesture::fullTime,
            {
            { // -O
                RefereeKeypoints::Keypoint::rightShoulder,
                RefereeKeypoints::Keypoint::rightElbow,
                {20, 110},
                {-120_deg, -60_deg}
            },
            { // -O
                RefereeKeypoints::Keypoint::rightShoulder,
                RefereeKeypoints::Keypoint::rightWrist,
                {50, 220},
                {-110_deg, -70_deg}
            },
            { //  O-
                RefereeKeypoints::Keypoint::leftShoulder,
                RefereeKeypoints::Keypoint::leftElbow,
                {20, 110},
                {60_deg, 120_deg}
            },
            { //  O-
                RefereeKeypoints::Keypoint::leftShoulder,
                RefereeKeypoints::Keypoint::leftWrist,
                {50, 220},
                {70_deg, 110_deg}
            }
            }
        },
        {
            RefereeGesture::Gesture::substitution,
            {
                { // <O
                    RefereeKeypoints::Keypoint::rightShoulder,
                    RefereeKeypoints::Keypoint::rightElbow,
                    {20, 110},
                    {-120_deg, 0_deg}
                },
                { // <O
                    RefereeKeypoints::Keypoint::rightElbow,
                    RefereeKeypoints::Keypoint::rightWrist,
                    {5, 110},
                    {60_deg, 165_deg}
                },
                { //  O>
                    RefereeKeypoints::Keypoint::leftShoulder,
                    RefereeKeypoints::Keypoint::leftElbow,
                    {20, 110},
                    {0_deg, 120_deg}
                },
                { //  O>
                    RefereeKeypoints::Keypoint::leftElbow,
                    RefereeKeypoints::Keypoint::leftWrist,
                    {5, 110},
                    {-165_deg, -60_deg}
                }
            }
        },
        {
            RefereeGesture::Gesture::goalBlue,
            {
                { // -O
                    RefereeKeypoints::Keypoint::rightShoulder,
                    RefereeKeypoints::Keypoint::rightElbow,
                    {20, 110},
                    {-120_deg, -60_deg}
                },
                { // -O
                    RefereeKeypoints::Keypoint::rightShoulder,
                    RefereeKeypoints::Keypoint::rightWrist,
                    {50, 220},
                    {-110_deg, -70_deg}
                }
            }
        },
        {
            RefereeGesture::Gesture::goalRed,
            {
                { //  O-
                    RefereeKeypoints::Keypoint::leftShoulder,
                    RefereeKeypoints::Keypoint::leftElbow,
                    {20, 110},
                    {60_deg, 120_deg}
                },
                { //  O-
                    RefereeKeypoints::Keypoint::leftShoulder,
                    RefereeKeypoints::Keypoint::leftWrist,
                    {50, 220},
                    {70_deg, 110_deg}
                }
            }
        }
    };

    RingBuffer<RefereeGesture::Gesture> history; /**< Recently detected gestures. */
    std::array<int, RefereeGesture::Gesture::count> histogram = {}; /**< How often was each gesture detected recently? */

    void detect_(CameraInfo::Camera whichCamera,
        const VisionInfoIn* info_in, 
        VisionInfoMiddle* info_middle,
        VisionInfoOut* info_out);

    void configure(Blackboard* blackboard);

    /**
     * Checks whether keypoints exist on both sides of the image.
     * The referee should stand in the middle of the image. Therefore, there must be
     * points on both sides.
     * @return Could this detection be the referee?
     */
    bool crossesMiddle(const CameraImage* cameraImage, RefereeKeypoints& keypoints) const;

    /**
     * Checks whether at least one point is below the horizon.
     * This should exclude people on grand stands.
     * @return At least one point below horizon?
     */
    bool pointsLowEnough(RefereeKeypoints& keypoints) const;

    /**
     * Get a keypoint returned by the network. The order to its left/right counterpart
     * is maintained under the assumption that the referee faces this robot.
     * @param keypoint The keypoint from the perspective of the observed referee.
     * @return The network output for the keypoint.
     */
    const RefereeKeypoints::Point& getOrdered(const RefereeKeypoints::Keypoint keypoint, RefereeKeypoints& keypoints) const;

    /**
     * Detect a gesture based on the network output.
     * @return The keypoint detected. Can be \c none.
     */
    RefereeGesture::Gesture detectGesture(RefereeKeypoints& keypoints) const;

    /**
     * Update the histogram of the recently detected gestures.
     * @param gesture The latest gesture detected. Can be \c none.
     */
    void updateHistogram(const RefereeGesture::Gesture gesture);

    /**
     * Compute the predominant gesture from the histogram of recently detected gestures.
     * @return The predominant gesture. Is \c none if no gesture is predominant enough.
     */
    RefereeGesture::Gesture getPredominantGesture() const;

    /**
     * Reset gesture history and histogram.
     */
    void resetHistogram() {
        history.clear();
        std::fill(histogram.begin(), histogram.end(), 0);
    }
};