#pragma once

#include <array>
#include <boost/function.hpp>
#include <boost/program_options/variables_map.hpp>
#include <cstdint>
#include <utility>
#include <vector>

#include "perception/vision/VisionDefinitions.hpp"
#include "perception/vision/fovea/Region.hpp"
#include "types/BallInfo.hpp"
#include "types/camera/AutoExposureWeightTable.hpp"
#include "types/camera/CameraImage.hpp"
#include "types/camera/CameraInfo.hpp"
#include "types/camera/CameraResolution.hpp"
#include "types/camera/CameraSettings.hpp"
#include "types/FieldFeatureInfo.hpp"
#include "types/RobotVisionInfo.hpp"
#include "types/field/FieldBoundary.hpp"
#include "types/vision/RefereeGesture.hpp"

// This header file is required for the Python buffer image version
#include <Python.h>

/**
 * Shared data Vision module will be sharing with others.
 */
struct VisionBlackboard {
    explicit VisionBlackboard();
    void readOptions(const boost::program_options::variables_map& config);

    /* Time the frame was captured */
    int64_t timestamp;

    /* Detected features */
    std::array<FieldBoundary, CameraInfo::Camera::NUM_CAMERAS> fieldBoundary;
    std::vector<BallInfo> balls;
    std::vector<FieldFeatureInfo> fieldFeatures;
    std::vector<RobotVisionInfo> robots;
    RefereeGesture refereeGesture;

    /* OLD - Detected features - Being Replaced */
    std::vector<RegionI> regions;

    /** Pointer to the current frame being processed by Vision */
    uint8_t const* topFrame;
    uint8_t const* botFrame;

    /** Current frame being processed by Vision */
    CameraImage topImage;
    CameraImage botImage;

    /** JPEG compression quality for serialization */
    int8_t topFrameJPEGQuality;
    int8_t botFrameJPEGQuality;

    /** Current camera settings on the robot */
    CameraInfo          topInfo;
    CameraResolution    topResolution;
    CameraSettings      topCameraSettings;
    AutoExposureWeightTable::Table topAutoExposureWeightTable;
    CameraInfo          botInfo;
    CameraResolution    botResolution;
    CameraSettings      botCameraSettings;
    AutoExposureWeightTable::Table botAutoExposureWeightTable;

    // Top Camera image in python buffer format
    PyObject* py_buffer;
};
