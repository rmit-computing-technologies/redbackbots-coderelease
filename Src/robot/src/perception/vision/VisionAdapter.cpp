#include "perception/vision/VisionAdapter.hpp"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <sys/time.h>        /* For gettimeofday */
#include <pthread.h>
#include <vector>

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/MotionBlackboard.hpp"
#include "blackboard/modules/StateEstimationBlackboard.hpp"
#include "blackboard/modules/SynchronisationBlackboard.hpp"
#include "blackboard/modules/VisionBlackboard.hpp"
#include "utils/Logger.hpp"
#include "utils/Timer.hpp"
#include "types/CombinedCameraSettings.hpp"
#include "types/CombinedFrame.hpp"

// This header file is now configured without the python2.7 qualifier
#include <Python.h>

using namespace std;
using namespace boost::algorithm;

extern int ADAPTIVE_THRESHOLDING_WINDOW_SIZE_TOP;
extern int ADAPTIVE_THRESHOLDING_WINDOW_SIZE_BOT;
extern int ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_TOP;
extern int ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_BOT;
int ADAPTIVE_THRESHOLDING_WINDOW_SIZE_TOP = 0;
int ADAPTIVE_THRESHOLDING_WINDOW_SIZE_BOT = 0;
int ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_TOP = 0;
int ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_BOT = 0;

VisionAdapter::VisionAdapter(Blackboard *bb)
   : Adapter(bb),
     vision_()
{
    combined_camera_ = new CombinedCamera(
        (blackboard->config)["vision.dumpframes"].as<bool>(),
        (blackboard->config)["vision.dumprate"].as<int>(),
        (blackboard->config)["vision.dumpfile"].as<string>()
    );

    combined_frame_ = boost::shared_ptr<CombinedFrame>();

    RegionI topRegion = vision_.getFullRegionTop();
    RegionI botRegion = vision_.getFullRegionBot();
    const Fovea *foveaTop = topRegion.getInternalFovea();
    const Fovea *foveaBot = botRegion.getInternalFovea();

    ADAPTIVE_THRESHOLDING_WINDOW_SIZE_TOP = (blackboard->config)["vision.top.adaptivethresholdingwindow"].as<int>();
    ADAPTIVE_THRESHOLDING_WINDOW_SIZE_BOT = (blackboard->config)["vision.bot.adaptivethresholdingwindow"].as<int>();
    ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_TOP = (blackboard->config)["vision.top.adaptivethresholdingpercent"].as<int>();
    ADAPTIVE_THRESHOLDING_WHITE_THRESHOLD_PERCENT_BOT = (blackboard->config)["vision.top.adaptivethresholdingpercent"].as<int>();

    writeTo(vision, topSaliency, (Colour*)foveaTop->getInternalColour());
    writeTo(vision, botSaliency, (Colour*)foveaBot->getInternalColour());
}

void VisionAdapter::tick() {
    Timer t;
    uint32_t time;

    /*
     * Camera Tick
     */
    llog(TRACE) << "Vision Camera Tick" << endl;
    t.restart();
    tickCamera();
    time = t.elapsed_us();
    if (time > 30000) {
        llog(TRACE) << "Vision Camera Tick: OK " << time << " us" << endl;
    } else {
        llog(TRACE) << "Vision Camera Tick: TOO LONG " << time << " us" << endl;
    }

    /*
     * Process Tick
     */
    llog(TRACE) << "Vision Process Tick" << endl;
    t.restart();
    tickProcess();
    time = t.elapsed_us();
    if (time > 30000) {
        llog(TRACE) << "Vision Process Tick: OK " << time << " us" << endl;
    } else {
        llog(TRACE) << "Vision Process Tick: TOO LONG " << time << " us" << endl;
    }

}

void VisionAdapter::tickCamera() {
    if (combined_camera_ == NULL) {
        llog(WARNING) << "No Camera provided to the VisionAdapter" << endl;
    } else {
        // llog(TRACE) << "VisionAdapter - Get camera frames" << std::endl;
        const uint8_t* point = combined_camera_->getFrameTop();
        // llog(TRACE) << "VisionAdapter - Got top frame" << std::endl;
        writeTo(vision, topFrame, point);
        // llog(TRACE) << "VisionAdapter - write to BB (point)" << std::endl;
        writeTo(vision, botFrame, combined_camera_->getFrameBottom());
        // llog(TRACE) << "VisionAdapter - write to BB (botFrame)" << std::endl;

        // TODO: Add comment explaining this
        // llog(TRACE) << "VisionAdapter - Create python object" << std::endl;
        // PyObject* py_buf = PyBuffer_FromMemory((uint8_t*) point, 2457600); // Python 2.7 API
        // TW: The char and uint8 should be the same length

        if (point != nullptr) {
            PyObject* py_buf = PyMemoryView_FromMemory((char *) point, 2457600, PyBUF_READ); // Interface has changed for Python3.8
            writeTo(vision, py_buffer, py_buf);
        }

        // Write the camera settings to the Blackboard
        // for syncing with OffNao's camera tab
        // llog(TRACE) << "VisionAdapter - Write camera settings to BB" << std::endl;
        CombinedCameraSettings settings = combined_camera_->getCameraSettings();
        writeTo(vision, topCameraSettings, settings.top_camera_settings);
        writeTo(vision, botCameraSettings, settings.bot_camera_settings);
    }
}

void VisionAdapter::tickProcess() {
    Timer t;
    VisionInfoIn info_in;

    // Used by the Python WallTimer.py
    // Might not belong here, just quick fixing Ready skill for now.
    struct timeval tv;
    gettimeofday(&tv, 0);

    int64_t vision_timestamp = tv.tv_sec * 1e6 + tv.tv_usec;

    // TODO Read current Pose from blakboard
    conv_rr_.pose = readFrom(motion, pose);
    conv_rr_.updateAngles(readFrom(motion, sensors));

    info_in.cameraToRR = conv_rr_;
    info_in.pose = conv_rr_.pose;
    info_in.robotPose = readFrom(stateEstimation, robotPos);

    // Set latestAngleX.
    info_in.latestAngleX =
              readFrom(motion, sensors).sensors[Sensors::InertialSensor_AngleX];

    // TODO ADD THIS IN
    conv_rr_.findEndScanValues();

    /*
     * Acquire blackboard lock
     */
    acquireLock(serialization);
    llog(TRACE) << "Vision tickProcess acquireLock(serialization) took " << t.elapsed_us()
      << " us" << endl;
    t.restart();

    /*
     * Reading from Blackboard into VisionBlackboardInfo
     */
    // NOTE: You can add things to info_in below by going
    // NOTE: info_in.robotDetection.sonar = readFrom(kinematics, sonarFiltered)

    info_in.top_camera_settings = readFrom(vision, topCameraSettings);
    info_in.bot_camera_settings = readFrom(vision, botCameraSettings);

    const uint8_t* topFrame = readFrom(vision, topFrame);
    const uint8_t* botFrame = readFrom(vision, botFrame);
    boost::shared_ptr<CombinedFrame> combined_frame_;
    combined_frame_ = boost::shared_ptr<CombinedFrame>(new CombinedFrame(
        topFrame,
        botFrame,
        conv_rr_,
        combined_frame_
    ));

    llog(TRACE) << "Vision reading images from blackboard took " << t.elapsed_us()
      << " us" << endl;

    t.restart();

    /*
     * Running Process Frame
     */
    llog(TRACE) << "Vision reading remaining data from blackboard took " << t.elapsed_us() << " us" << endl;
    t.restart();

    pthread_yield();
    usleep(1); // force sleep incase yield sucks

    VisionInfoOut info_out;
    if (topFrame != nullptr && botFrame != nullptr) {
        info_out = vision_.processFrame(*(combined_frame_.get()), info_in);
    } else {
        llog(INFO) << "NULL camera image, not processing vision" << std::endl;
    }

    llog(TRACE) << "Vision processFrame() took " << t.elapsed_us() << " us" << endl;
    t.restart();

    /*
     * Writing Results back to blackboard
     */

    // NOTE: You can add things back to the blackboard by going
    // NOTE: writeTo(vision, [blackboard var name], info_out.[info_in var name])

    writeTo (vision, timestamp,       vision_timestamp        );
    // Note that these regions will not be able to access their underlying pixel
    // data.
    writeTo (vision, regions,         info_out.regions        );
    writeTo (vision, balls,           info_out.balls          );
    writeTo (vision, robots,          info_out.robots         );
    writeTo (vision, fieldFeatures,   info_out.features       );
    writeTo (vision, fieldBoundaries, info_out.boundaries     );

    releaseLock(serialization);
    llog(TRACE) << "Vision releaseLock(serialization) - writing back to blackboard took " << t.elapsed_us() << " us" << endl;
}
