#include "types/camera/CameraSettings.hpp"

CameraSettings::CameraSettings() {
    for (int i = 0; i != CameraSetting::NUM_CAMERA_SETTINGS; ++i) {
        settings[i] = 0;
    }

    hflip = 0;
    vflip = 0;

    // Old settings
    whiteBalance = 0;
    aeTargetAvgLuma = 0;
    aeTargetAvgLumaDark = 0;
    aeTargetGain = 0;
    aeMinVirtGain = 0;
    aeMaxVirtGain = 0;
    aeMinVirtAGain = 0;
    aeMaxVirtAGain = 0;
    aeTargetExposure = 0;
    aeUseWeightTable = false;
};

std::string CameraSettings::enumToString(CameraSetting setting) {
    std::string string;
    if (setting == CameraSetting::AUTO_EXPOSURE) {
        string = "AUTO_EXPOSURE";
    } else if (setting == CameraSetting::BRIGHTNESS) {
        string = "BRIGHTNESS";
    } else if (setting == CameraSetting::EXPOSURE) {
        string = "EXPOSURE";
    } else if (setting == CameraSetting::GAIN) {
        string = "GAIN";
    } else if (setting == CameraSetting::AUTO_WHITE_BALANCE) {
        string = "AUTO_WHITE_BALANCE";
    } else if (setting == CameraSetting::AUTO_FOCUS) {
        string = "AUTO_FOCUS";
    } else if (setting == CameraSetting::FOCUS) {
        string = "FOCUS";
    } else if (setting == CameraSetting::AUTO_HUE) {
        string = "AUTO_HUE";
    } else if (setting == CameraSetting::HUE) {
        string = "HUE";
    } else if (setting == CameraSetting::SATURATION) {
        string = "SATURATION";
    } else if (setting == CameraSetting::CONTRAST) {
        string = "CONTRAST";
    } else if (setting == CameraSetting::SHARPNESS) {
        string = "SHARPNESS";
    } else if (setting == CameraSetting::RED_GAIN) {
        string = "RED_GAIN";
    } else if (setting == CameraSetting::GREEN_GAIN) {
        string = "GREEN_GAIN";
    } else if (setting == CameraSetting::BLUE_GAIN) {
        string = "BLUE_GAIN";
    } else {
        string = "unknown_setting";
    }

    return string;
}

// CameraSettings& CameraSettings::operator=(const CameraSettings& other) {
//     for (int i = 0; i != CameraSetting::NUM_CAMERA_SETTINGS; ++i) {
//         settings[i] = other.settings[i];
//     }

//     hflip = other.hflip;
//     vflip = other.vflip;

//     // Old settings
//     whiteBalance = other.whiteBalance;
//     aeTargetAvgLuma = other.aeTargetAvgLuma;
//     aeTargetAvgLumaDark = other.aeTargetAvgLumaDark;
//     aeTargetGain = other.aeTargetGain;
//     aeMinVirtGain = other.aeMinVirtGain;
//     aeMaxVirtGain = other.aeMaxVirtGain;
//     aeMinVirtAGain = other.aeMinVirtAGain;
//     aeMaxVirtAGain = other.aeMaxVirtAGain;
//     aeTargetExposure = other.aeTargetExposure;
//     aeUseWeightTable = other.aeUseWeightTable;
    
//     return *this;
// }
