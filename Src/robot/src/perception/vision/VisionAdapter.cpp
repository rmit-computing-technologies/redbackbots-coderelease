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
#include "types/camera/CombinedCameraSettings.hpp"
#include "types/camera/CombinedFrame.hpp"
#include "types/camera/CameraImage.hpp"
#include "utils/Logger.hpp"
#include "utils/Timer.hpp"
#include "utils/debug/CameraImageDumper.hpp"


VisionAdapter::VisionAdapter(Blackboard *bb)
   : Adapter(bb),
     vision_(bb)
{
    combined_camera_ = new CombinedCamera();
    imageDumper = new CameraImageDumper(bb);

    // combined_frame_ = boost::shared_ptr<CombinedFrame>();
}

VisionAdapter::~VisionAdapter() {
    delete combined_camera_;
    delete imageDumper;
}

void VisionAdapter::tick() {
    Timer t;
    uint32_t time;

    /*
     * Camera Tick
     */
    llog(TRACE) << "Vision Camera Tick" << std::endl;
    t.restart();
    tickCamera();
    time = t.elapsed_us();
    if (time > (30000 * 2)) {
        llog(TRACE) << "Vision Camera Tick: OK " << time << " us" << std::endl;
    } else {
        llog(TRACE) << "Vision Camera Tick: TOO LONG " << time << " us" << std::endl;
    }

    /*
     * Process Tick
     */
    llog(TRACE) << "Vision Process Tick" << std::endl;
    t.restart();
    tickProcess();
    time = t.elapsed_us();
    if (time > (30000 * 2)) {
        llog(TRACE) << "Vision Process Tick: OK " << time << " us" << std::endl;
    } else {
        llog(TRACE) << "Vision Process Tick: TOO LONG " << time << " us" << std::endl;
    }

}

void VisionAdapter::tickCamera() {
    if (combined_camera_ == NULL) {
        llog(WARNING) << "No Camera provided to the VisionAdapter" << std::endl;
    } else {
        acquireLock(serialization);

        // Write of CameraImages is contained within the NaoCameraProvider
        llog(TRACE) << "VisionAdapter - Get camera frames" << std::endl;
        combined_camera_->getFrameTop();
        combined_camera_->getFrameBottom();
        llog(TRACE) << "VisionAdapter - got frames" << std::endl;

        releaseLock(serialization);

        // If debugging, dump images (top image for now)
        imageDumper->dumpImage(readFrom(vision, topImage));
    }
}

void VisionAdapter::tickProcess() {
    Timer t;

    // Used by the Python WallTimer.py
    // Might not belong here, just quick fixing Ready skill for now.
    struct timeval tv;
    gettimeofday(&tv, 0);

    // 'Synchronised' timestamp of vision state (with camera images)
    int64_t vision_timestamp = tv.tv_sec * 1e6 + tv.tv_usec;

    // TODO (TW): REMOVE: - only used by fovea generation
    // conv_rr_.pose = readFrom(motion, pose);
    // conv_rr_.updateAngles(readFrom(motion, sensors));
    // conv_rr_.findEndScanValues();

    // Vision info for this frame
    VisionInfoIn info_in;

    // Store kinematics and pose information
    info_in.kinematicPose = readFrom(motion, pose);
    info_in.worldPose = readFrom(stateEstimation, robotPos);
    info_in.playerNum = readFrom(gameController, player_number);
    // info_in.sensorValues = readFrom(motion, sensors);

    // Set latestAngleX.
    info_in.latestAngleX = readFrom(motion, sensors).sensors[Sensors::InertialSensor_AngleX];

    /*
     * Acquire blackboard lock
     */
    acquireLock(serialization);
    llog(TRACE) << "Vision tickProcess acquireLock(serialization) took " << t.elapsed_us()
                << " us" << std::endl;
    t.restart();

    /*
     * Reading from Blackboard into VisionBlackboardInfo
     */
    // NOTE: You can add things to info_in below by going
    // NOTE: info_in.robotDetection.sonar = readFrom(kinematics, sonarFiltered)

    // Set Camera Image and information
    info_in.image[CameraInfo::Camera::top] = &(readFrom(vision, topImage));
    info_in.image[CameraInfo::Camera::bot] = &(readFrom(vision, botImage));
    info_in.cameraInfo[CameraInfo::Camera::top] = readFrom(vision, topInfo);
    info_in.cameraInfo[CameraInfo::Camera::bot] = readFrom(vision, botInfo);
    info_in.cameraResolution[CameraInfo::Camera::top] = readFrom(vision, topResolution);
    info_in.cameraResolution[CameraInfo::Camera::bot] = readFrom(vision, botResolution);
    info_in.cameraSettings[CameraInfo::Camera::top] = readFrom(vision, topCameraSettings);
    info_in.cameraSettings[CameraInfo::Camera::bot] = readFrom(vision, botCameraSettings);

    // TODO (TW): Remove when bhuman vision port is done
    // boost::shared_ptr<CombinedFrame> combined_frame_;
    // combined_frame_ = boost::shared_ptr<CombinedFrame>(new CombinedFrame(
    //     readFrom(vision, topFrame),
    //     readFrom(vision, botFrame),
    //     conv_rr_,
    //     combined_frame_
    // ));

    llog(TRACE) << "Vision reading images from blackboard took " << t.elapsed_us()
                << " us" << std::endl;

    t.restart();

    /*
     * Running Process Frame
     */
    llog(TRACE) << "Vision reading remaining data from blackboard took " << t.elapsed_us() << " us" << std::endl;
    t.restart();

    pthread_yield();
    usleep(1); // force sleep in case yield sucks

    // Run the vision algorithms on the current combined frame
    VisionInfoOut info_out = vision_.processFrame(info_in);

    llog(TRACE) << "Vision processFrame() took " << t.elapsed_us() << " us" << std::endl;
    t.restart();

    /*
     * Writing Results back to blackboard
     */

    // NOTE: You can add things back to the blackboard by going
    // NOTE: writeTo(vision, [blackboard var name], info_out.[info_in var name])

    writeTo(vision, timestamp,         vision_timestamp            );
    writeTo(vision, fieldBoundary,     info_out.fieldBoundary      );
    writeTo(vision, balls,             info_out.balls              );
    writeTo(vision, fieldFeatures,     info_out.features           );
    writeTo(vision, robots,            info_out.robots             );
    writeTo(vision, refereeGesture,    info_out.refereeGesture     );

    /* OLD WRITES */
    // Note that these regions will not be able to access their underlying pixel data.
    // writeTo(vision, regions,         info_out.regions        );
    // writeTo(vision, balls,           info_out.balls          );
    // writeTo(vision, robots,          info_out.robots         );
    // writeTo(vision, fieldFeatures,   info_out.features       );
    // writeTo(vision, fieldBoundaries, info_out.boundaries     );

    releaseLock(serialization);
    llog(TRACE) << "Vision releaseLock(serialization) - writing back to blackboard took " << t.elapsed_us() << " us" << std::endl;
}
