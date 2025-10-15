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

#include "perception/vision/camera/NaoCameraProvider.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/VisionBlackboard.hpp"
#include "perception/kinematics/RobotPose.hpp"
#include "perception/vision/camera/NaoCameraV6.hpp"
#include "types/camera/CameraImage.hpp"
#include "utils/math/angles.hpp"
#include "utils/math/MD5.hpp"
#include "utils/Timer.hpp"

#include <sstream>

#include <Python.h>

bool NaoCameraProvider::resetPending = false;

NaoCameraProvider::NaoCameraProvider(Blackboard *blackboard,
                                     const std::string filename,
                                     CameraInfo::Camera whichCamera)
    : blackboard(blackboard), filename(filename), whichCamera(whichCamera),
      maxWaitForImage(2000), resetDelay(2000)
{
    // TODO: get resetDelay from blackboard config. Default to 2000ms

    // Human readable camera name (and string for options/config)
    if (whichCamera == CameraInfo::Camera::top) {
        cameraChoice = "camera.top";
    } else if (whichCamera == CameraInfo::Camera::bot) {
        cameraChoice = "camera.bot";
    }

    // Get camera settings from config
    readCameraIntrinsics();
    readCameraCalibrations();
    readCameraResolutions();
    readCameraSettings();
    readAutoExposureTable();

    // Setup & configure camera
    setupCamera();
    
    // Update blackboard with initial values
    if (whichCamera == CameraInfo::Camera::top) {
        writeTo(vision, topInfo, cameraInfo);
        writeTo(vision, topResolution, cameraResolution);
        writeTo(vision, topCameraSettings, cameraSettings);
        writeTo(vision, topAutoExposureWeightTable, autoExposureWeightTable);
    } else {
        writeTo(vision, botInfo, cameraInfo);
        writeTo(vision, botResolution, cameraResolution);
        writeTo(vision, botCameraSettings, cameraSettings);
        writeTo(vision, botAutoExposureWeightTable, autoExposureWeightTable);
    }
}

NaoCameraProvider::~NaoCameraProvider() {
    blackboard = nullptr;
    if (camera != nullptr) {
        delete camera;
    }
}

void NaoCameraProvider::readCameraIntrinsics() {
    std::string cameraName = cameraChoice;

    cameraIntrinsics.openingAngleHeight = DEG2RAD(blackboard->config[(cameraName + ".intrinsic.openingAngleHeight").c_str()].as<float>());
    cameraIntrinsics.openingAngleWidth = DEG2RAD(blackboard->config[(cameraName + ".intrinsic.openingAngleWidth").c_str()].as<float>());
    cameraIntrinsics.opticalCenter.x() = blackboard->config[(cameraName + ".intrinsic.opticalcentre.x").c_str()].as<float>();
    cameraIntrinsics.opticalCenter.y() = blackboard->config[(cameraName + ".intrinsic.opticalcentre.y").c_str()].as<float>();
}

void NaoCameraProvider::readCameraCalibrations() {
    std::string cameraName = cameraChoice;

    cameraCalibrations.cameraRotationCorrections.x = DEG2RAD(blackboard->config[(cameraName + ".extrinsic.rotationCorrection.x").c_str()].as<float>());
    cameraCalibrations.cameraRotationCorrections.y = DEG2RAD(blackboard->config[(cameraName + ".extrinsic.rotationCorrection.y").c_str()].as<float>());
    cameraCalibrations.cameraRotationCorrections.z = DEG2RAD(blackboard->config[(cameraName + ".extrinsic.rotationCorrection.z").c_str()].as<float>());
}

void NaoCameraProvider::readCameraResolutions() {
    std::string cameraName = cameraChoice;

    std::string resolution = blackboard->config[(cameraName + ".resolution").c_str()].as<std::string>();
    if (resolution == "w320h240") {
        cameraResolution = CameraResolution::w320h240;
    } else if (resolution == "w640h480") {
        cameraResolution = CameraResolution::w640h480;
    } else if (resolution == "w1280h960") {
        cameraResolution = CameraResolution::w1280h960;
    }

    llog(TRACE) << cameraName << " resolution: " <<  resolution << " (enum: " << cameraResolution  << ")" << std::endl;
}

void NaoCameraProvider::readCameraSettings() {
    std::string cameraName = cameraChoice;

    // Read dynamic settings
    cameraSettings.settings[CameraSettings::CameraSetting::AUTO_EXPOSURE] = blackboard->config[(cameraName + ".exposureauto").c_str()].as<int>();
    cameraSettings.settings[CameraSettings::CameraSetting::BRIGHTNESS] = blackboard->config[(cameraName + ".brightness").c_str()].as<int>();
    cameraSettings.settings[CameraSettings::CameraSetting::EXPOSURE] = blackboard->config[(cameraName + ".exposure").c_str()].as<int>();
    cameraSettings.settings[CameraSettings::CameraSetting::GAIN] = blackboard->config[(cameraName + ".gain").c_str()].as<int>();
    cameraSettings.settings[CameraSettings::CameraSetting::AUTO_WHITE_BALANCE] = blackboard->config[(cameraName + ".autowhitebalance").c_str()].as<int>();
    cameraSettings.settings[CameraSettings::CameraSetting::AUTO_FOCUS] = blackboard->config[(cameraName + ".autofocus").c_str()].as<int>();
    cameraSettings.settings[CameraSettings::CameraSetting::FOCUS] = blackboard->config[(cameraName + ".focusabsolute").c_str()].as<int>();
    cameraSettings.settings[CameraSettings::CameraSetting::AUTO_HUE] = blackboard->config[(cameraName + ".autohue").c_str()].as<int>();
    cameraSettings.settings[CameraSettings::CameraSetting::HUE] = blackboard->config[(cameraName + ".hue").c_str()].as<int>();
    cameraSettings.settings[CameraSettings::CameraSetting::SATURATION] = blackboard->config[(cameraName + ".saturation").c_str()].as<int>();
    cameraSettings.settings[CameraSettings::CameraSetting::CONTRAST] = blackboard->config[(cameraName + ".contrast").c_str()].as<int>();
    cameraSettings.settings[CameraSettings::CameraSetting::SHARPNESS] = blackboard->config[(cameraName + ".sharpness").c_str()].as<int>();
    cameraSettings.settings[CameraSettings::CameraSetting::RED_GAIN] = blackboard->config[(cameraName + ".gainred").c_str()].as<int>();
    cameraSettings.settings[CameraSettings::CameraSetting::GREEN_GAIN] = blackboard->config[(cameraName + ".gaingreen").c_str()].as<int>();
    cameraSettings.settings[CameraSettings::CameraSetting::BLUE_GAIN] = blackboard->config[(cameraName + ".gainblue").c_str()].as<int>();


    // Read special settings
    cameraSettings.hflip = blackboard->config[(cameraName + ".hflip").c_str()].as<int>();
    cameraSettings.vflip = blackboard->config[(cameraName + ".vflip").c_str()].as<int>();

    // Old camera settings
    cameraSettings.whiteBalance = blackboard->config[(cameraName + ".whitebalance").c_str()].as<int>();
    cameraSettings.aeTargetAvgLuma = blackboard->config[(cameraName + ".aetargetavgluma").c_str()].as<int>();
    cameraSettings.aeTargetAvgLumaDark = blackboard->config[(cameraName + ".aetargetavglumadark").c_str()].as<int>();
    cameraSettings.aeTargetGain = blackboard->config[(cameraName + ".aetargetgain").c_str()].as<int>();
    cameraSettings.aeMinVirtGain = blackboard->config[(cameraName + ".aeminvirtgain").c_str()].as<int>();
    cameraSettings.aeMaxVirtGain = blackboard->config[(cameraName + ".aemaxvirtgain").c_str()].as<int>();
    cameraSettings.aeMinVirtAGain = blackboard->config[(cameraName + ".aeminvirtagain").c_str()].as<int>();
    cameraSettings.aeMaxVirtAGain = blackboard->config[(cameraName + ".aemaxvirtagain").c_str()].as<int>();
    cameraSettings.aeTargetExposure = blackboard->config[(cameraName + ".aetargetexposure").c_str()].as<int>();
    cameraSettings.aeUseWeightTable = blackboard->config[(cameraName + ".aeuseweighttable").c_str()].as<bool>();
    cameraSettings.aeWeightTableX1 = blackboard->config[(cameraName + ".aeweighttablex1").c_str()].as<float>();
    cameraSettings.aeWeightTableX2 = blackboard->config[(cameraName + ".aeweighttablex2").c_str()].as<float>();
    cameraSettings.aeWeightTableY1 = blackboard->config[(cameraName + ".aeweighttabley1").c_str()].as<float>();
    cameraSettings.aeWeightTableY2 = blackboard->config[(cameraName + ".aeweighttabley2").c_str()].as<float>();
}

void NaoCameraProvider::readAutoExposureTable() {
    std::string cameraName = cameraChoice;
    std::string tableStr = blackboard->config[(cameraName + ".autoexposureweighttable").c_str()].as<std::string>();

    // llog(DEBUG) << "Auto exposure weight table for " << cameraName << " : " << tableStr << std::endl;
    // Expected format of string is: 16 numbers comma separated. Eg:
    //      0,0,0,0,1,1,1,1,5,8,8,5,5,5,5,5
    // Split string
    std::vector<uint8_t> tableValues;
    if (tableStr != "") {
        std::stringstream ss(tableStr);
        std::string token;
        while (std::getline(ss, token, ',')) {
            try {
                int v = std::stoi(token);
                tableValues.push_back((uint8_t) (v));
            } catch (const std::exception& e) {
                break;
            }
        }
    }

    AutoExposureWeightTable::Table table;
    if (tableValues.size() == 16) {
        for(int i = 0; i != tableValues.size(); ++i) {
            table(i) = tableValues[i];
        }
    } else {
        // Default values
        if (whichCamera == CameraInfo::Camera::top) {
            table << 
                0, 0, 0, 0,
                1, 1, 1, 1,
                5, 8, 8, 5,
                5, 5, 5, 5
            ;
        } else {
            table << 
                5, 5, 5, 5,
                5, 8, 8, 5,
                5, 8, 8, 5,
                5, 5, 5, 5
            ;
        }
    }
    autoExposureWeightTable = table;

    // llog(INFO) << "************** " << cameraName << " auto exposure weight table : ";
    // for (int i = 0; i != 16; ++i) {
    //     llog(INFO) << (int) (autoExposureWeightTable(i)) << ",";
    // }
    // llog(INFO) << std::endl;
}

void NaoCameraProvider::setupCamera() {
    cameraWidth = 320;
    cameraHeight = 240;
    switch (cameraResolution) {
        case CameraResolution::w1280h960:
            cameraWidth     = 1280;
            cameraHeight    = 960;
            break;
        case CameraResolution::w640h480:
            cameraWidth     = 640;
            cameraHeight    = 480;
            break;
        case CameraResolution::defaultRes:
        case CameraResolution::w320h240:
            cameraWidth     = 320;
            cameraHeight    = 240;
            break;
        default:
            llog(ERROR) << "Unknown resolution format, defaulting to " 
                        << cameraWidth << "x" << cameraHeight << std::endl;
    }

    // Set Camera type
    cameraInfo.camera = whichCamera;

    // set info dimensions
    cameraInfo.height = cameraHeight;
    cameraInfo.width = cameraWidth;

    // set opening angle
    cameraInfo.openingAngleWidth = cameraIntrinsics.openingAngleWidth;
    cameraInfo.openingAngleHeight = cameraIntrinsics.openingAngleHeight;

    // set optical center
    cameraInfo.opticalCenter.x() = cameraIntrinsics.opticalCenter.x() * cameraInfo.width;
    cameraInfo.opticalCenter.y() = cameraIntrinsics.opticalCenter.y() * cameraInfo.height;

    // set rotation corrections
    cameraInfo.cameraCalibrations = cameraCalibrations;

    // update focal length
    cameraInfo.updateFocalLength();

    // Set static focal length values where required elsewhere
    // TODO (TW): Check if this should use the INV versions
    if (whichCamera == CameraInfo::Camera::top) {
        RobotPose::topFocalLength = cameraInfo.focalLength;
    } else {
        RobotPose::botFocalLength = cameraInfo.focalLength;
    }

    // Configure and start-up the camera
    assert(camera == nullptr);
    camera = new NaoCameraV6(filename, whichCamera, cameraWidth, cameraHeight, cameraSettings, autoExposureWeightTable);
}

bool NaoCameraProvider::processResolutionRequest() {
    // TODO (TW): Implement
    return false;
}


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
//                 cameraResolution.resolutions[whichCamera] = whichCamera == CameraInfo::upper
//                                                                        ? CameraResolutionRequest::w640h480
//                                                                        : CameraResolutionRequest::w320h240;
//             return true;
//         case CameraResolutionRequest::w320h240:
//         case CameraResolutionRequest::w640h480:
//         case CameraResolutionRequest::w1280h960:
//             cameraResolution.resolutions[whichCamera] = theCameraResolutionRequest.resolutions[whichCamera];
//             return true;
//         default:
//             FAIL("Unknown resolution.");
//             return false;
//         }
//     }
//     else
//         return false;
// }

CameraResolution NaoCameraProvider::getResolution() {
    return cameraResolution;
}

bool NaoCameraProvider::hasImage() {
    if (resetPending) {
        return false;
    } else if (camera != nullptr) {
        return camera->hasImage();
    }
    return false;
}

const uint8_t* NaoCameraProvider::takeImage() {
    // llog(TRACE) << "Get image for " << cameraChoice << std::endl;

    // Load latest blackboard information
    if (whichCamera == CameraInfo::Camera::top) {
        cameraResolution        = readFrom(vision, topResolution);
        cameraSettings          = readFrom(vision, topCameraSettings);
        autoExposureWeightTable = readFrom(vision, topAutoExposureWeightTable);
    } else {
        cameraResolution        = readFrom(vision, botResolution);
        cameraSettings          = readFrom(vision, botCameraSettings);
        autoExposureWeightTable = readFrom(vision, botAutoExposureWeightTable);
    }

    // Timer if camera is taking too long
    Timer t;

    // Reset camera if required
    if (camera->resetRequired) {
        resetPending = true;
    }
    if (resetPending) {
        delete camera;
        camera = nullptr;

        NaoCameraV6::resetCamera();
        llog(INFO) << "Camera reset" << std::endl;
        resetPending = false;

        setupCamera();
    }

    // Make sure we have the latest camera image
    if (camera->hasImage()) {
        camera->releaseImage();
    }

    // Update resolution
    // TODO (TW): Implement processResolutionRequest() - currently returns false
    if (processResolutionRequest()) {
        delete camera;
        camera = nullptr;
        setupCamera();
    }

    // Retrieve image
    while (!camera->hasImage()) {
        cameraOk = camera->captureNew(maxWaitForImage);

        if (!cameraOk) {
            llog(INFO) << "Camera broken - resetting" << std::endl;
            break;
        }
    }

    // Process image
    const uint8_t* image = nullptr;
    if (camera->hasImage()) {
        image = camera->getImage();
        updateBlackboard(image);
    } else if (t.elapsed_ms() > resetDelay) {
        // Try resetting here if the camera read image above fails
        //     so we can avoid a null image on this loop

        // Delete camera (and close file handle)
        delete camera;
        camera = nullptr;
        setupCamera();
    }

    // Update settings
    if (camera->hasImage()) {
        camera->setSettings(cameraSettings, autoExposureWeightTable);
        camera->writeCameraSettings();
    }
    // llog(TRACE) << CameraInfo::enumCameraToString(whichCamera) << ": total " << t.elapsed_ms() << " ms" << std::endl;
  
    return image;
}

void NaoCameraProvider::updateBlackboard(const uint8_t* image) {
    // Write image to blackboard (Old uint8 version of image)
    if (whichCamera == CameraInfo::Camera::top) {
        writeTo(vision, topFrame, image);

        // For the top camera, write the python buffer to blackboard
        PyObject* py_buf = PyMemoryView_FromMemory((char *) image, 2457600, PyBUF_READ);
        writeTo(vision, py_buffer, py_buf);
    } else {
        writeTo(vision, botFrame, image);
    }

    // Use timer for image timestamp as it gives the current systime
    Timer t;
    unsigned timestamp = static_cast<long long>(camera->getTimestamp() / 1000) > static_cast<long long>(t.elapsed_ms())
                             ? static_cast<unsigned>(camera->getTimestamp() / 1000 - t.elapsed_ms())
                             : 100000;

    // Update image on blackboard
    if (camera->hasImage()) {
        // RECALL that the image uses a YUYV (2-pixel wide) format with Chroma Subsampling in the width. 
        // That is, each 'pixel' in YUYV is 2 actual pixels.
        // Therefore the 'internal width' of the image is 'divided by 2', 
        // which is then multiplied by 2 (when iterating) with the YUYV Pixel Type to get the 'true' image width.
        // See: https://en.wikipedia.org/wiki/Chroma_subsampling

        if (whichCamera == CameraInfo::Camera::top) {
            blackboard->vision->topImage.setImage(cameraWidth / 2, cameraHeight, const_cast<unsigned char *>(image), std::max(lastImageTimestamp + 1, timestamp));
        } else if (whichCamera == CameraInfo::Camera::bot) {
            blackboard->vision->botImage.setImage(cameraWidth / 2, cameraHeight, const_cast<unsigned char *>(image), std::max(lastImageTimestamp + 1, timestamp));

            // Check for distortion issues in the bottom camera
            if (blackboard->vision->botImage.timestamp - timestampLastRowChange > 3000) {
                timestampLastRowChange = blackboard->vision->botImage.timestamp;
                currentRow = Random::uniformInt(1, cameraHeight - 2);
                rowBuffer.clear();
            }
            std::string res = MD5().digestMemory(
                reinterpret_cast<uint8_t *>(blackboard->vision->botImage[currentRow]), 
                                            cameraHeight * sizeof(CameraImage::PixelType)
            );
            rowBuffer.push_front(res);

            int appearances = 0;
            for (auto i = rowBuffer.begin(); i != rowBuffer.end(); ++i) {
                if (*i == res && ++appearances > 25) {
                    llog(ERROR) << "Probably encountered a distorted image in the lower camera - resetting" << std::endl;
                    camera->resetRequired = true;
                    return;
                }
            }
        }
    }

    // For debugging purposes - uncomment if needed to check white-balancing
    // camera->doAutoWhiteBalance();
    // camera->readCameraSettings();
    // cameraSettings = camera->getCameraSettings();
    // if (whichCamera == CameraInfo::Camera::top) {
    //     writeTo(vision, topCameraSettings, cameraSettings);
    // } else {
    //     writeTo(vision, botCameraSettings, cameraSettings);
    // }

    // Update timestamp tracking
    lastImageTimestamp = timestamp;
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
