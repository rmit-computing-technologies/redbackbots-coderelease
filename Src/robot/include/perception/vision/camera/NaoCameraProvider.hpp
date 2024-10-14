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

#include "perception/vision/camera/Camera.hpp"
#include "types/CameraSettings.hpp"

#include <string>

#define VIDEO_TOP "/dev/video-top"
#define VIDEO_BOTTOM "/dev/video-bottom"

// Forward Declare Blackboard
class Blackboard;

// Forward Declare NaoCameraV6
class NaoCameraV6;

namespace AL {
   /**
    *Format of the image
   */
   const int kQQVGA = 0;  // 160*120
   const int kQVGA  = 1;  // 320*240
   const int kVGA   = 2;  // 640*480
   const int k16VGA = 4;  //2560*1920
   const int k1920p = k16VGA;  //2560*1920
   const int k720p = 5;  //1280*720
   const int k1080p = 6;  //1920*1080
   const int kQQQVGA = 7;  // 80*60
   const int kQQQQVGA = 8;  // 40*30

   // Instantiate all format variables for the V6
   const int k4VGA  = 3;  //1280*960
   const int k960p  = k4VGA;  //deprecated
}

class NaoCameraProvider : public Camera {
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
                     const std::string cameraChoice,
                     const int format = AL::k960p);
    virtual ~NaoCameraProvider();

    /**
     * The method returns whether a new image is available.
     * @return Is an new image available?
     */
    bool hasImage();

    /**
     * Get the image and convert it to the requested colourSpace.
     * Current defaults to V4L2_PIX_FMT_YUYV for V4L library
     *
     * @return a pointer to the image if the image was fetched successfully.
     * NULL, otherwise.
     */
    virtual const uint8_t *get(const __u32 colourSpace = V4L2_PIX_FMT_YUYV);

    bool setControl(const uint32_t id, const int32_t value);
    static void setAutoExposureTarget(int fd, uint8_t high);

    // Camera setting fields.
    CameraSettings cameraSettings;

private:
    // Camera being provided by this class
    NaoCameraV6* camera = nullptr;
    
    // Filename of device path to camera
    std::string filename;

    // Keep track of the current camera choice as a human-readable string
    std::string cameraChoice;

    // Image format
    int format;
    int cameraWidth;
    int cameraHeight;

    // Configuration
    int maxWaitForImage;
    int resetDelay;

    volatile bool cameraOk = true;
    
    // True if *this* camera needs to reset
    bool resetPending;

    // Static reset communication between camera
    //     True if the *other* camera tells *this* camera is needs to reset
    static bool triggerSharedReset;

    void readCameraSettings(Blackboard *blackboard);
    void readCameraSettings(Blackboard *blackboard, CameraSettings &settings, std::string cameraName);

    void setupCamera();

};
