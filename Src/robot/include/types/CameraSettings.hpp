#pragma once

class CameraSettings {
public:
    CameraSettings();

    unsigned int hflip;
    unsigned int vflip;
    unsigned int brightness;
    unsigned int contrast;
    unsigned int saturation;
    unsigned int hue;
    unsigned int sharpness;
    unsigned int backlightCompensation;
    unsigned int exposure;
    unsigned int gain;
    unsigned int whiteBalance;
    unsigned int exposureAuto;
    unsigned int autoWhiteBalance;
    unsigned int autoFocus;
    unsigned int focusAbsolute;
    unsigned int exposureAlgorithm;
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
