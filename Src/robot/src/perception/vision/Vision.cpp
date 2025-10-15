
#include "perception/vision/Vision.hpp"

#include "blackboard/modules/DebuggerBlackboard.hpp"
#include "blackboard/modules/debuger/VisionDebuggerBlackboard.hpp"
#include "perception/vision/VisionInfoMiddle.hpp"

// Detectors
#include "perception/vision/detector/FieldBoundaryDetector.hpp"
#include "perception/vision/middleinfo/ECImageMiddleInfo.hpp"
#include "perception/vision/middleinfo/ScanGridMiddleInfo.hpp"
#include "perception/vision/middleinfo/RelativeFieldColorsMiddleInfo.hpp"
#include "perception/vision/middleinfo/ScanLineMiddleInfo.hpp"
#include "perception/vision/middleinfo/PenaltyMarkRegionsMiddleInfo.hpp"
#include "perception/vision/middleinfo/PenaltyMarkMiddleInfo.hpp"
#include "perception/vision/middleinfo/LineAndCircleMiddleInfo.hpp"
#include "perception/vision/middleinfo/IntersectionCandidatesMiddleInfo.hpp"
#include "perception/vision/middleinfo/IntersectionsMiddleInfo.hpp"
#include "perception/vision/middleinfo/BallSpotsMiddleInfo.hpp"
#include "perception/vision/middleinfo/BOPMiddleInfo.hpp"
#include "perception/vision/middleinfo/RobotMiddleInfo.hpp"
#include "perception/vision/middleinfo/RobotLowerMiddleInfo.hpp"
#include "perception/vision/detector/BallDetector.hpp"
#include "perception/vision/detector/FieldFeatureDetector.hpp"
#include "perception/vision/detector/RobotDetector.hpp"
#include "perception/vision/other/KinematicsPoseTester.hpp"
#include "perception/vision/referee/KeypointsDetector.hpp"
#include "perception/vision/referee/RefereeGestureDetector.hpp"

#include "utils/Logger.hpp"
#include "soccer.hpp"

#include <asmjit/asmjit.h>


// Enum for setting the collection of algorithm timers
enum DetectorTimers {
    TIMER_GENERAL = 0,
    TIMER_FIELD_BOUNDARY,
    TIMER_FIELD_FEATURES,
    TIMER_BALL_DETECTOR,
    TIMER_ROBOT_DETECTOR,
    TIMER_MIDDLE_INFO,
    TIMER_REFEREE_DETECTOR,

    NUM_TIMERS
};

Vision::Vision(Blackboard* blackboard) :
    blackboard(blackboard)
{

    for (int i = 0; i != DetectorTimers::NUM_TIMERS; ++i) {
        timers.push_back(0);
    }

    // Create vision info
    info_in_ = new VisionInfoIn();
    info_middle_ = new VisionInfoMiddle();
    info_out_ = new VisionInfoOut();

    // Initialise default/static info elements
    info_middle_->fieldFeatureLocations = FieldFeatureLocations();

    // JitRuntime
    asmjitRuntime = new asmjit::JitRuntime();

    // Configure algorithms and ordering
    setupAlgorithms_(blackboard);

    llog(INFO) << "Vision Created" << std::endl;
}

Vision::~Vision() {
    detectors_.clear();
    referee_detectors_.clear();

    delete info_out_;
    delete info_middle_;
    delete info_in_;

    delete asmjitRuntime;

    llog(INFO) << "Vision Destroyed" << std::endl;
}

VisionInfoOut Vision::processFrame(const VisionInfoIn& info_in) {
    Timer t;
    uint32_t time;

    // Reset to process frame
    resetVisionInfo();
    *info_in_ = info_in;

    // Set Camera to RR in vision out
    // info_out_->cameraToRR = info_in_->cameraToRR;
    info_out_->kinematicPose = info_in_->kinematicPose;

    // Track the number of frames we've seen.
    ++frameCount;

    /*
     * Full Finders, Region Finders, and Detectors
     */
    t.restart();
    if (offNao) {

    } else {
        runAlgorithms_();
    }

    // Write debug information from middle info - these are compiled out if NDEBUG is present
    // Other blackboard population is done by other code parts and should not be done here
    // This is ONLY for debugging purposes
    writeTo_debugger(vision, scanGrid, info_middle_->scanGrid);
    writeTo_debugger(vision, colorScanLineRegionsHorizontal, info_middle_->colorScanLineRegionsHorizontal);
    writeTo_debugger(vision, colorScanLineRegionsVerticalClipped, info_middle_->colorScanLineRegionsVerticalClipped);
    writeTo_debugger(vision, penaltyMarkRegions, info_middle_->penaltyMarkRegions);
    writeTo_debugger(vision, ballSpots, info_middle_->ballSpots);
    writeTo_debugger(vision, lineSpots, info_middle_->lineSpots);
    writeTo_debugger(vision, candidates, info_middle_->candidates);
    writeTo_debugger(vision, candidatesBefore, info_middle_->candidatesBefore);
    writeTo_debugger(vision, candidatesAfter, info_middle_->candidatesAfter);
    writeTo_debugger(vision, circleCandidates, info_middle_->circleCandidates);
    writeTo_debugger(vision, centreCircle, info_middle_->centreCircle);
    writeTo_debugger(vision, intersectionCandidates, info_middle_->intersectionCandidates);
    writeTo_debugger(vision, planeSpots, info_middle_->planeSpots);
    writeTo_debugger(vision, isWhiteSpots, info_middle_->IsWhiteSpots);
    writeTo_debugger(vision, robotsImage, info_middle_->robotsImage);
    writeTo_debugger(vision, refereeKeypoints, info_middle_->refereeKeypoints);

    // Log the 1000 frame average vision timings.
    if (frameCount == 1000) {
        // Reset the frame count.
        frameCount = 0;

        // Print the timings.
        llog(DEBUG) << std::endl << "VISION TIMINGS AVG (us)" << std::endl;
        llog(DEBUG) << "Average middle info time: " << ((float) timers[DetectorTimers::TIMER_MIDDLE_INFO])/1000.0f << std::endl;
        llog(DEBUG) << "Average field boundary time: " << ((float) timers[DetectorTimers::TIMER_FIELD_BOUNDARY])/1000.0f << std::endl;
        llog(DEBUG) << "Average field features time: " << ((float) timers[DetectorTimers::TIMER_FIELD_FEATURES])/1000.0f << std::endl;
        llog(DEBUG) << "Average ball detector time: " << ((float) timers[DetectorTimers::TIMER_BALL_DETECTOR])/1000.0f << std::endl;
        llog(DEBUG) << "Average robot detector time: " << ((float) timers[DetectorTimers::TIMER_ROBOT_DETECTOR])/1000.0f << std::endl;
        llog(DEBUG) << "Average referee detector time: " << ((float) timers[DetectorTimers::TIMER_REFEREE_DETECTOR])/1000.0f << std::endl;
        
        // Reset timers (and give total average)
        int total = 0;
        for (int i = 0; i != DetectorTimers::NUM_TIMERS; ++i) {
            total += timers[i];
            timers[i] = 0;
        }
        llog(DEBUG) << "Average total processFrame time: " << ((float) total) / 1000.0f << std::endl;
    }

    return *info_out_;
}

void Vision::setupAlgorithms_(Blackboard* blackboard) {
    std::shared_ptr<Detector> detector = nullptr;

    // Field Boundary
    detector = std::make_shared<FieldBoundaryDetector>(blackboard, asmjitRuntime);
    detectors_.push_back(std::make_pair(detector, DetectorTimers::TIMER_FIELD_BOUNDARY));

    // Middle Info processors
    // ECImage
    detector = std::make_shared<ECImageMiddleInfo>(blackboard, asmjitRuntime);
    detectors_.push_back(std::make_pair(detector, DetectorTimers::TIMER_MIDDLE_INFO));

    // BOP (ball spots, obstacles, penalty mark regions) for bottom camera
    detector = std::make_shared<BOPMiddleInfo>(blackboard, asmjitRuntime);
    detectors_.push_back(std::make_pair(detector, DetectorTimers::TIMER_MIDDLE_INFO));

    // Robot detectors
    detector = std::make_shared<RobotMiddleInfo>(blackboard, asmjitRuntime);
    detectors_.push_back(std::make_pair(detector, DetectorTimers::TIMER_ROBOT_DETECTOR));
    
    detector = std::make_shared<RobotLowerMiddleInfo>(blackboard);
    detectors_.push_back(std::make_pair(detector, DetectorTimers::TIMER_ROBOT_DETECTOR));

    // Scan Grid
    detector = std::make_shared<ScanGridMiddleInfo>(blackboard);
    detectors_.push_back(std::make_pair(detector, DetectorTimers::TIMER_MIDDLE_INFO));

    // Relative Field Colors
    detector = std::make_shared<RelativeFieldColorsMiddleInfo>(blackboard);
    detectors_.push_back(std::make_pair(detector, DetectorTimers::TIMER_MIDDLE_INFO));

    // Scan Line
    detector = std::make_shared<ScanLineMiddleInfo>(blackboard);
    detectors_.push_back(std::make_pair(detector, DetectorTimers::TIMER_MIDDLE_INFO));

    // Line Perceptor
    detector = std::make_shared<LineAndCircleMiddleInfo>(blackboard);
    detectors_.push_back(std::make_pair(detector, DetectorTimers::TIMER_MIDDLE_INFO));

    // Intersection Candidates
    detector = std::make_shared<IntersectionCandidatesMiddleInfo>(blackboard);
    detectors_.push_back(std::make_pair(detector, DetectorTimers::TIMER_MIDDLE_INFO));

    // Intersections
    detector = std::make_shared<IntersectionsMiddleInfo>(blackboard, asmjitRuntime);
    detectors_.push_back(std::make_pair(detector, DetectorTimers::TIMER_MIDDLE_INFO));

    // Potential Penalty Mark Regions
    detector = std::make_shared<PenaltyMarkRegionsMiddleInfo>(blackboard);
    detectors_.push_back(std::make_pair(detector, DetectorTimers::TIMER_MIDDLE_INFO));

    // Ball spots
    detector = std::make_shared<BallSpotsMiddleInfo>(blackboard);
    detectors_.push_back(std::make_pair(detector, DetectorTimers::TIMER_MIDDLE_INFO));

    // Ball detector
    detector = std::make_shared<BallDetector>(blackboard, asmjitRuntime);
    detectors_.push_back(std::make_pair(detector, DetectorTimers::TIMER_BALL_DETECTOR));

    // Penalty Marks
    detector = std::make_shared<PenaltyMarkMiddleInfo>(blackboard, asmjitRuntime);
    detectors_.push_back(std::make_pair(detector, DetectorTimers::TIMER_MIDDLE_INFO));

    // Field features
    detector = std::make_shared<FieldFeatureDetector>(blackboard);
    detectors_.push_back(std::make_pair(detector, DetectorTimers::TIMER_FIELD_FEATURES));

    // Robots (rr)
    detector = std::make_shared<RobotDetector>(blackboard);
    detectors_.push_back(std::make_pair(detector, DetectorTimers::TIMER_ROBOT_DETECTOR));

    // // Tester
    // detector = std::make_shared<KinematicsPoseTester>(blackboard);
    // detectors_.push_back(std::make_pair(detector, DetectorTimers::TIMER_MIDDLE_INFO));

    /* --- (MO) CURRENTLY TESTING VISUAL REFEREE IN MAIN VISION THREAD - NEED TO DISABLE WHEN NOT IN STANDBY --- */ 
    detector = std::make_shared<KeypointsDetector>(blackboard, asmjitRuntime);
    referee_detectors_.push_back(std::make_pair(detector, DetectorTimers::TIMER_REFEREE_DETECTOR));

    detector = std::make_shared<RefereeGestureDetector>(blackboard);
    referee_detectors_.push_back(std::make_pair(detector, DetectorTimers::TIMER_REFEREE_DETECTOR));
    /* ------------------------------------------------------------------------------------------------------------------------------------- */

    llog(INFO) << NDEBUG_LOGSYMB << "Setup of Algorithms done" << std::endl;
}

void Vision::runAlgorithms_() {
    for (auto algorithm : detectors_) {
        algorithm.first->timer.restart();
        algorithm.first->detect(info_in_, info_middle_, info_out_);

        // Update timer
        timers[algorithm.second] += algorithm.first->timer.elapsed_us();
    }

    if(readFrom(gameController, gameState) == STATE_STANDBY) {
        for (auto algorithm : referee_detectors_) {
            algorithm.first->timer.restart();
            algorithm.first->detect(info_in_, info_middle_, info_out_);

            // Update timer
            timers[algorithm.second] += algorithm.first->timer.elapsed_us();
        }
    }
}

void Vision::resetVisionInfo() {
    // Capture previous tick information
    // ... nothing at the moment

    // Reset Middle Info and Vision Out
    for (auto algorithm : detectors_) {
        algorithm.first->resetMiddleInfo(info_middle_);
        algorithm.first->resetVisionOut(info_out_);
    }

    for (auto algorithm : referee_detectors_) {
        algorithm.first->resetMiddleInfo(info_middle_);
        algorithm.first->resetVisionOut(info_out_);
    }
}
