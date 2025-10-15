#pragma once

#include <array>
#include <string>

class CameraSettings {
public:
    // Dynamic/Modifiable camera settings
    enum CameraSetting {            
        AUTO_EXPOSURE = 0,            /**< Auto exposure mode (0: on, 1: off). */
        BRIGHTNESS,                   /**< The brightness the auto exposure tries to achieve in the range [-255 .. 255]. */
        EXPOSURE,                     /**< The exposure time in the range [0 .. 1048575] when auto exposure is disabled. */
        GAIN,                         /**< The gain level in the range [0 .. 1023]. */
        AUTO_WHITE_BALANCE,           /**< 1: Use auto white balance, 0: disable auto white balance, current white balance is frozen. */
        AUTO_FOCUS,                   /**< 1: Use auto focus, 0: disable auto focus. */
        FOCUS,                        /**< The focus in the range [0 .. 250] in steps of 25 when auto focus is disabled. */
        AUTO_HUE,                     /**< 1: Use auto hue, 0: disable auto hue. */
        HUE,                          /**< The hue in the range [-180 .. 180] when auto hue is disabled. */
        SATURATION,                   /**< The saturation in the range [0 .. 255]. */
        CONTRAST,                     /**< The contrast in the range [0 .. 255]. */
        SHARPNESS,                    /**< The sharpness in the range [0 .. 9]. */
        RED_GAIN,                     /**< The red gain in the range [0 .. 4095] when autoWhiteBalance is disabled. */
        GREEN_GAIN,                   /**< The green gain in the range [0 .. 4095] when autoWhiteBalance is disabled. */
        BLUE_GAIN,                    /**< The blue gain in the range [0 .. 4095] when autoWhiteBalance is disabled. */

        // Records number of elements in the enum (based on C++ enum representations)
        NUM_CAMERA_SETTINGS,
    };

    // Construct default Camera settings
    CameraSettings();

    // Assignment operator
    // CameraSettings& operator=(const CameraSettings& other);

    // Special camera settings, which do not get updated one the camera is initialised
    int hflip;                        /**< 1: Flip image along horizontal axis, 0: Do not flip. */
    int vflip;                        /**< 1: Flip image along vertical axis, 0: Do not flip. */

    // Track settings
    std::array<int, CameraSetting::NUM_CAMERA_SETTINGS> settings;

    // Human readable string for the enum
    static std::string enumToString(CameraSetting setting);

    // Old camera settings setup
    unsigned int whiteBalance;
    unsigned int aeTargetAvgLuma;
    unsigned int aeTargetAvgLumaDark;
    unsigned int aeTargetGain;
    unsigned int aeMinVirtGain;
    unsigned int aeMaxVirtGain;
    unsigned int aeMinVirtAGain;
    unsigned int aeMaxVirtAGain;
    unsigned int aeTargetExposure;
    bool         aeUseWeightTable;
    float        aeWeightTableX1;
    float        aeWeightTableX2;
    float        aeWeightTableY1;
    float        aeWeightTableY2;
};
