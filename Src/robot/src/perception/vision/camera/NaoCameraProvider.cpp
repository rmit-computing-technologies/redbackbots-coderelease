/**
 * This file implements a module that handles the communication with the two
 * cameras. This implementation starts a separate thread to avoid blocking
 * calls to slow down processing.
 *
 * Modified for RedBackBots from the
 * original implementation by B-Human (2021 code release)
 *
 * @author Colin Graf
 * @author Thomas Röfer
 * @author RedBackBots
 */

#include "blackboard/Blackboard.hpp"
#include "perception/vision/camera/NaoCameraProvider.hpp"
#include "perception/vision/camera/NaoCameraV6.hpp"
#include "utils/Timer.hpp"

// Static shared trigger communication
bool NaoCameraProvider::triggerSharedReset = false;


NaoCameraProvider::NaoCameraProvider(Blackboard *blackboard,
                                     const std::string filename,
                                     const std::string cameraChoice,
                                     const int format)
    : filename(filename), cameraChoice(cameraChoice), format(format),
      resetPending(false) {

    // TODO: get resetDelay from blackboard config
    // Default to 2000ms
    maxWaitForImage = 2000;
    resetDelay = 2000;

    // Get camera settings
    readCameraSettings(blackboard);

    // Setup & configure camera
    setupCamera();
}

NaoCameraProvider::~NaoCameraProvider() {
    if (camera != nullptr) {
        delete camera;
    }
}

void NaoCameraProvider::readCameraSettings(Blackboard *blackboard) {
    readCameraSettings(blackboard, cameraSettings, cameraChoice);
}

void NaoCameraProvider::readCameraSettings(Blackboard *blackboard,
        CameraSettings &settings, std::string cameraName) {
    settings.hflip = blackboard->config[(cameraName + ".hflip").c_str()].as<int>();
    settings.vflip = blackboard->config[(cameraName + ".vflip").c_str()].as<int>();
    settings.brightness = blackboard->config[(cameraName + ".brightness").c_str()].as<int>();
    settings.contrast = blackboard->config[(cameraName + ".contrast").c_str()].as<int>();
    settings.saturation = blackboard->config[(cameraName + ".saturation").c_str()].as<int>();
    settings.hue = blackboard->config[(cameraName + ".hue").c_str()].as<int>();
    settings.sharpness = blackboard->config[(cameraName + ".sharpness").c_str()].as<int>();
    settings.backlightCompensation = blackboard->config[(cameraName + ".backlightcompensation").c_str()].as<int>();
    settings.exposure = blackboard->config[(cameraName + ".exposure").c_str()].as<int>();
    settings.gain = blackboard->config[(cameraName + ".gain").c_str()].as<int>();
    settings.whiteBalance = blackboard->config[(cameraName + ".whitebalance").c_str()].as<int>();
    settings.exposureAuto = blackboard->config[(cameraName + ".exposureauto").c_str()].as<int>();
    settings.autoWhiteBalance = blackboard->config[(cameraName + ".autowhitebalance").c_str()].as<int>();
    settings.autoFocus = blackboard->config[(cameraName + ".autofocus").c_str()].as<int>();
    settings.focusAbsolute = blackboard->config[(cameraName + ".focusabsolute").c_str()].as<int>();
    settings.exposureAlgorithm = blackboard->config[(cameraName + ".exposurealgorithm").c_str()].as<int>();
    settings.aeTargetAvgLuma = blackboard->config[(cameraName + ".aetargetavgluma").c_str()].as<int>();
    settings.aeTargetAvgLumaDark = blackboard->config[(cameraName + ".aetargetavglumadark").c_str()].as<int>();
    settings.aeTargetGain = blackboard->config[(cameraName + ".aetargetgain").c_str()].as<int>();
    settings.aeMinVirtGain = blackboard->config[(cameraName + ".aeminvirtgain").c_str()].as<int>();
    settings.aeMaxVirtGain = blackboard->config[(cameraName + ".aemaxvirtgain").c_str()].as<int>();
    settings.aeMinVirtAGain = blackboard->config[(cameraName + ".aeminvirtagain").c_str()].as<int>();
    settings.aeMaxVirtAGain = blackboard->config[(cameraName + ".aemaxvirtagain").c_str()].as<int>();
    settings.aeTargetExposure = blackboard->config[(cameraName + ".aetargetexposure").c_str()].as<int>();
    settings.aeUseWeightTable = blackboard->config[(cameraName + ".aeuseweighttable").c_str()].as<bool>();
    settings.aeWeightTableX1 = blackboard->config[(cameraName + ".aeweighttablex1").c_str()].as<float>();
    settings.aeWeightTableX2 = blackboard->config[(cameraName + ".aeweighttablex2").c_str()].as<float>();
    settings.aeWeightTableY1 = blackboard->config[(cameraName + ".aeweighttabley1").c_str()].as<float>();
    settings.aeWeightTableY2 = blackboard->config[(cameraName + ".aeweighttabley2").c_str()].as<float>();

}

void NaoCameraProvider::setupCamera() {
    // set resolution
    cameraWidth = 320;
    cameraHeight = 240;
    switch (format) {
        case AL::k960p:
            cameraWidth     = 1280;
            cameraHeight    = 960;
            break;
        case AL::kVGA:
            cameraWidth     = 640;
            cameraHeight    = 480;
            break;
        case AL::kQVGA:
            cameraWidth     = 320;
            cameraHeight    = 240;
            break;
        default:
            llog(ERROR) << "Unknown format, defaulting to " << cameraWidth << "x" << cameraHeight << std::endl;
    }

    // Configure and start-up the camera
    assert(camera == nullptr);
    camera = new NaoCameraV6(filename, cameraChoice, cameraWidth, cameraHeight, cameraSettings);
}

const uint8_t *NaoCameraProvider::get(const __u32 colourSpace) {
    if (colourSpace != V4L2_PIX_FMT_YUYV) {
        throw std::runtime_error("only yuv422 is supported!");
    }

    // llog(TRACE) << "Get image for " << cameraChoice << std::endl;
    Timer t;

    // Check for camera reset
    if (camera->resetRequired) {
        resetPending = true;
    }

    // Reset Camera if required
    if (resetPending || triggerSharedReset) {
        if (triggerSharedReset) {
            llog(INFO) << cameraChoice << ": triggerSharedReset" << std::endl;
        }
        // We've now triggered shared reset, so clear flag
        triggerSharedReset = false;

        // Remove the old camera
        delete camera;
        camera = nullptr;

        // If this camera needs to reset, then clear the I2C bus
        if (resetPending) {
            // Only do this if this camera needs to trigger
            //      if reset is triggered from the other place, then don't do this
            NaoCameraV6::resetCamera();
            llog(INFO) << cameraChoice << ": Camera reset (#1)" << std::endl;
            triggerSharedReset = true;
            resetPending = false;
        }

        setupCamera();
    }

    // Ensure we get the latest possible image
    if (camera->hasImage()) {
        camera->releaseImage();
    }

    while (!camera->hasImage()) {
        cameraOk = camera->captureNew(maxWaitForImage);
        if (!cameraOk) {
            llog(INFO) << cameraChoice << ": Camera is broken (#1)" << std::endl;
            break;
        }
    }

    const uint8_t *image = nullptr;
    if (camera->hasImage()) {
        // Get image
        t.restart();
        image = camera->getImage();
    } else if (t.elapsed_ms() > resetDelay) {
        // Try resetting here if the camera read image above fails
        //     so we can avoid a null image on this loop

        // Delete camera (and close file handle)
        delete camera;
        camera = nullptr;

        // Send reset
        NaoCameraV6::resetCamera();
        llog(INFO) << cameraChoice << ": Camera reset (#2)" << std::endl;
        resetPending = false;
        triggerSharedReset = true;

        // Setup the camera
        setupCamera();

        // Try to reach the image again
        t.restart();
        while (!camera->hasImage()) {
            cameraOk = camera->captureNew(maxWaitForImage);
            if (!cameraOk) {
                llog(INFO) << cameraChoice << ": Camera broken (#2)" << std::endl;
                break;
            }
        }

        if (camera->hasImage()) {
            image = camera->getImage();
            llog(INFO) << cameraChoice << ": Successfully got image after reset (#2)" << std::endl;
        }
    }

    if (image != nullptr) {
    //     llog(DEBUG) << "Image returning from NaoCamera: " << (void *)image << std::endl;
    } else {
        llog(INFO) << cameraChoice << ": Image not retrieved: " << (void *)image << std::endl;
    }

    // image = nullptr; // Test for null image handling
    return image;
}


bool NaoCameraProvider::setControl(const uint32_t controlId,
                           const int32_t controlValue) {
    bool retVal = false;
    if (camera != nullptr) {
        retVal = camera->setControl(controlId, controlValue);
    } else {
        llog(ERROR) << "Cannot set control without camera" << std::endl;
    }
    return retVal;
}

void NaoCameraProvider::setAutoExposureTarget(int fd, uint8_t high) {
    NaoCameraV6::setAutoExposureTarget(fd, high);
}

bool NaoCameraProvider::hasImage() {
    if (resetPending) {
        return false;
    } else if (camera != nullptr) {
        return camera->hasImage();
    }
    return false;
}
