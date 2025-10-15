/**
 * This file declares a module that handles the communication with the two
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

#pragma once

#include "types/camera/AutoExposureWeightTable.hpp"
#include "types/camera/CameraImage.hpp"
#include "types/camera/CameraInfo.hpp"
#include "types/camera/CameraIntrinsics.hpp"
#include "types/camera/CameraCalibrations.hpp"
#include "types/camera/CameraResolution.hpp"
#include "types/camera/CameraSettings.hpp"
#include "types/RingBuffer.h"
#include "utils/random.hpp"

#include <string>

#define VIDEO_TOP "/dev/video-top"
#define VIDEO_BOTTOM "/dev/video-bottom"

// Forward Declare Blackboard
class Blackboard;

// Forward Declare NaoCameraV6
class NaoCameraV6;


class NaoCameraProvider {
public:
    /**
     * Constructor
     * opens the device, calibrates it, and sets it up for streaming
     *
     * @param filename the device or file to get images from
     * @param method the io method to use
     * @see IOMethod
     * @param format the format of the image
     * @see AL::kVGA
     * @param cameraChoice - 'camera.top' or 'camera.bottom'
     */
    NaoCameraProvider(Blackboard *blackboard,
                      const std::string filename,
                      CameraInfo::Camera whichCamera);
    virtual ~NaoCameraProvider();

    bool setControl(const uint32_t id, const int32_t value);
    static void setAutoExposureTarget(int fd, uint8_t high);

    // Return camera properties
    CameraResolution getResolution();

    // Returns whether a new image is available
    bool hasImage();

    // Retrieve the next available image from the camera
    const uint8_t* takeImage();

    // Camera setting fields.
    CameraSettings cameraSettings;

private:
    // Blackboard ref
    Blackboard* blackboard;
    
    // Camera being provided by this class
    NaoCameraV6* camera = nullptr;
    
    // Filename of device path to camera
    std::string filename;

    // Camera top/bottom
    CameraInfo::Camera whichCamera;

    // Current camera choice as a human-readable string
    std::string cameraChoice;

    // Camera properties
    CameraInfo cameraInfo;
    CameraIntrinsics cameraIntrinsics;
    CameraCalibrations cameraCalibrations;
    CameraResolution cameraResolution;
    CameraResolution lastResolutionRequest = CameraResolution::defaultRes;
    AutoExposureWeightTable::Table autoExposureWeightTable;

    // Camera status
    volatile bool cameraOk = true;

    // Track if the camera requires a reset
    static bool resetPending;

    // Image format
    int cameraWidth;
    int cameraHeight;

    // Configuration
    int maxWaitForImage;
    int resetDelay;

    // For checking image
    RingBuffer<std::string, 120> rowBuffer;
    unsigned int currentRow = 0;
    unsigned int timestampLastRowChange = 0;
    unsigned int lastImageTimestamp = 0;

    // Read camera defaults from blackboard
    void readCameraIntrinsics();
    void readCameraCalibrations();
    void readCameraResolutions();
    void readCameraSettings();
    void readAutoExposureTable();

    // Configure the Camera
    void setupCamera();

    // If there is a resolution request, change the camera resolution
    bool processResolutionRequest();

    // Update Blackboard with latest camera image, and camera settings
    void updateBlackboard(const uint8_t* image);

};
