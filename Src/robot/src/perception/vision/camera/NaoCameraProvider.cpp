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

// thread_local NaoCameraProvider *NaoCameraProvider::theInstance = nullptr;
// #ifdef CAMERA_INCLUDED
// Semaphore NaoCameraProvider::performingReset = Semaphore(1);
bool NaoCameraProvider::resetPending = false;
// #endif

NaoCameraProvider::NaoCameraProvider(Blackboard *blackboard,
                                     const std::string filename,
                                     const std::string cameraChoice,
                                     const int format)
    : filename(filename), cameraChoice(cameraChoice), format(format) {

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

    llog(TRACE) << "Get image for " << cameraChoice << std::endl;
    Timer t;

    // Check for camera reset
    if (camera->resetRequired) {
        resetPending = true;
    }

    // Reset Camera if required
    if (resetPending) {
        delete camera;
        camera = nullptr;
        if (resetPending) {
            NaoCameraV6::resetCamera();
            llog(INFO) << "Camera reset" << std::endl;
            resetPending = false;
        }
        setupCamera();
        // TODO:
        // B-human had continue here - should we wait for image?
        //continue
    }

    // Ensure we get the latest possible image
    if (camera->hasImage()) {
        camera->releaseImage();
    }

    while (!camera->hasImage()) {
        cameraOk = camera->captureNew(maxWaitForImage);
        if (!cameraOk) {
            llog(INFO) << "Camera broken" << std::endl;
            break;
        }
    }

    const uint8_t *image = nullptr;
    if (camera->hasImage()) {
        // Get image
        t.restart();
        image = camera->getImage();
    } else if (t.elapsed_ms() > resetDelay) {
        delete camera;
        camera = new NaoCameraV6(filename, cameraChoice, cameraWidth, cameraHeight, cameraSettings);
        t.restart();
        while (!camera->hasImage()) {
            cameraOk = camera->captureNew(maxWaitForImage);
            if (!cameraOk) {
                llog(INFO) << "Camera broken" << std::endl;
                break;
            }
        }
        if (camera->hasImage()) {
            image = camera->getImage();
        }
    }

    if (image != nullptr) {
        llog(DEBUG) << "Image returning from NaoCamera: " << (void *)image << std::endl;
    //     writeFrame(image);
    } else {
        llog(DEBUG) << "Image not retrieved: " << (void *)image << std::endl;
    }

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




// void NaoCameraProvider::update(CameraImage &theCameraImage)
// {
// #ifdef CAMERA_INCLUDED
//     unsigned timestamp = static_cast<long long>(camera->getTimestamp() / 1000) > static_cast<long long>(Time::getSystemTimeBase())
//                              ? static_cast<unsigned>(camera->getTimestamp() / 1000 - Time::getSystemTimeBase())
//                              : 100000;
//     if (camera->hasImage())
//     {
//         theCameraImage.setReference(cameraInfo.width / 2, cameraInfo.height, const_cast<unsigned char *>(camera->getImage()), std::max(lastImageTimestamp + 1, timestamp));

//         if (theCameraImage.timestamp - timestampLastRowChange > 3000)
//         {
//             timestampLastRowChange = theCameraImage.timestamp;
//             currentRow = Random::uniformInt(1, cameraInfo.height - 2);
//             rowBuffer.clear();
//         }
//         std::string res = MD5().digestMemory(reinterpret_cast<unsigned char *>(theCameraImage[currentRow]), cameraInfo.width * sizeof(CameraImage::PixelType));
//         rowBuffer.push_front(res);

//         int appearances = 0;
//         for (auto i = rowBuffer.begin(); i != rowBuffer.end(); ++i)
//             if (*i == res && ++appearances > 25)
//             {
//                 OUTPUT_ERROR("Probably encountered a distorted image!");
//                 camera->resetRequired = true;
//                 return;
//             }
//     }
//     else if (theCameraImage.isReference())
//     {
//         theCameraImage.setResolution(cameraInfo.width / 2, cameraInfo.height);
//         theCameraImage.timestamp = std::max(lastImageTimestamp + 1, timestamp);
//     }

//     if (whichCamera == CameraInfo::upper)
//     {
//         DEBUG_RESPONSE_ONCE("module:NaoCameraProvider:doWhiteBalanceUpper")
//         camera->doAutoWhiteBalance();
//         DEBUG_RESPONSE_ONCE("module:NaoCameraProvider:readCameraSettingsUpper")
//         camera->readCameraSettings();
//     }
//     else
//     {
//         DEBUG_RESPONSE_ONCE("module:NaoCameraProvider:doWhiteBalanceLower")
//         camera->doAutoWhiteBalance();
//         DEBUG_RESPONSE_ONCE("module:NaoCameraProvider:readCameraSettingsLower")
//         camera->readCameraSettings();
//     }
//     lastImageTimestampLL = camera->getTimestamp();

//     ASSERT(theCameraImage.timestamp >= lastImageTimestamp);
//     lastImageTimestamp = theCameraImage.timestamp;
// #else
//     theCameraImage.setResolution(cameraInfo.width / 2, cameraInfo.height);
//     theCameraImage.timestamp = Time::getCurrentSystemTime();
// #endif // CAMERA_INCLUDED
// }

// void NaoCameraProvider::update(JPEGImage &jpegImage)
// {
//     jpegImage = theCameraImage;
// }

// void NaoCameraProvider::update(CameraInfo &cameraInfo)
// {
//     cameraInfo = this->cameraInfo;
// }

// void NaoCameraProvider::update(CameraStatus &cameraStatus)
// {
//     if (!cameraOk)
//     {
//         if (cameraStatus.ok)
//         {
//             ANNOTATION("NaoCameraProvider", "Could not acquire new image.");
//             SystemCall::playSound("sirene.wav");
//             SystemCall::say("Camera broken");
//         }
// #ifdef NDEBUG
//         else if (!SystemCall::soundIsPlaying())
//         {
//             SystemCall::playSound("sirene.wav");
//             SystemCall::say("Camera broken");
//         }
// #endif
//     }

//     cameraStatus.ok = cameraOk;
// }

// bool NaoCameraProvider::processResolutionRequest()
// {
//     if (SystemCall::getMode() == SystemCall::Mode::physicalRobot && theCameraResolutionRequest.resolutions[whichCamera] != lastResolutionRequest)
//     {
//         lastResolutionRequest = theCameraResolutionRequest.resolutions[whichCamera];
//         switch (theCameraResolutionRequest.resolutions[whichCamera])
//         {
//         case CameraResolutionRequest::noRequest:
//             return false;
//         case CameraResolutionRequest::defaultRes:
//             if (!readCameraResolution())
//                 cameraResolutionRequest.resolutions[whichCamera] = whichCamera == CameraInfo::upper
//                                                                        ? CameraResolutionRequest::w640h480
//                                                                        : CameraResolutionRequest::w320h240;
//             return true;
//         case CameraResolutionRequest::w320h240:
//         case CameraResolutionRequest::w640h480:
//         case CameraResolutionRequest::w1280h960:
//             cameraResolutionRequest.resolutions[whichCamera] = theCameraResolutionRequest.resolutions[whichCamera];
//             return true;
//         default:
//             FAIL("Unknown resolution.");
//             return false;
//         }
//     }
//     else
//         return false;
// }


bool NaoCameraProvider::hasImage()
{
    if (resetPending) {
        return false;
    } else if (camera != nullptr) {
        return camera->hasImage();
    }
    return false;
}

// void NaoCameraProvider::takeImages()
// {
// #ifdef CAMERA_INCLUDED
//     BH_TRACE_INIT(whichCamera == CameraInfo::upper ? "UpperNaoCameraProvider" : "LowerNaoCameraProvider");
//     Thread::nameCurrentThread("NaoCameraProvider");
//     thread.setPriority(11);
//     unsigned imageReceived = Time::getRealSystemTime();
//     while (thread.isRunning())
//     {
//         if (camera->resetRequired)
//             resetPending = true;
//         if (resetPending)
//         {
//             delete camera;
//             camera = nullptr;
//             performingReset.wait();
//             if (resetPending)
//             {
//                 NaoCamera::resetCamera();
//                 SystemCall::playSound(headName.c_str());
//                 SystemCall::say("Camera reset");
//                 resetPending = false;
//             }
//             setupCamera();
//             performingReset.post();
//             continue;
//         }

//         takeNextImage.wait();

//         if (camera->hasImage())
//             camera->releaseImage();

//         // update resolution
//         if (processResolutionRequest())
//         {
//             delete camera;
//             camera = nullptr;
//             setupCamera();
//         }

//         while (!camera->hasImage())
//         {
//             cameraOk = camera->captureNew(maxWaitForImage);

//             if (!cameraOk)
//             {
//                 BH_TRACE_MSG("Camera broken");
//                 break;
//             }
//         }

//         if (camera->hasImage())
//             imageReceived = Time::getRealSystemTime();
//         else if (Time::getRealTimeSince(imageReceived) > resetDelay)
//         {
//             delete camera;
//             camera = new NaoCamera(whichCamera == CameraInfo::upper ? "/dev/video-top" : "/dev/video-bottom",
//                                    cameraInfo.camera,
//                                    cameraInfo.width, cameraInfo.height, whichCamera == CameraInfo::upper,
//                                    theCameraSettings.cameras[whichCamera], theAutoExposureWeightTable.tables[whichCamera]);
//             imageReceived = Time::getRealSystemTime();
//         }

//         if (camera->hasImage())
//             camera->setSettings(theCameraSettings.cameras[whichCamera], theAutoExposureWeightTable.tables[whichCamera]);

//         imageTaken.post();

//         if (camera->hasImage())
//             camera->writeCameraSettings();
//     }
// #endif
// }
