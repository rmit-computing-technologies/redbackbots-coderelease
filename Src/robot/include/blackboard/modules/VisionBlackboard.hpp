#pragma once

#include <boost/function.hpp>
#include <boost/program_options/variables_map.hpp>
#include <stdint.h>
#include <vector>
#include <utility>

#include "perception/vision/VisionDefinitions.hpp"
#include "perception/vision/Region.hpp"
#include "types/BallInfo.hpp"
#include "types/CameraSettings.hpp"
#include "types/FieldBoundaryInfo.hpp"
#include "types/FieldFeatureInfo.hpp"
#include "types/RobotVisionInfo.hpp"

// This header file is now configured without the python2.7 qualifier
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
    std::vector<BallInfo> balls;
    std::vector<RobotVisionInfo> robots;
    std::vector<FieldBoundaryInfo> fieldBoundaries;
    std::vector<FieldFeatureInfo> fieldFeatures;
    std::vector<RegionI> regions;

    /** Saliency scan */
    Colour *topSaliency;
    Colour *botSaliency;

    /** Pointer to the current frame being processed by Vision */
    uint8_t const* topFrame;
    uint8_t const* botFrame;

    /** JPEG compression for serialization */
    int8_t topFrameJPEGQuality;
    int8_t botFrameJPEGQuality;

    /** Current camera settings on the robot */
    CameraSettings topCameraSettings;
    CameraSettings botCameraSettings;

    // We need to send these over to offnao for the manual camera pose
   float horizontalFieldOfView;
   float verticalFieldOfView;

    // TODO: Add comment explaining this
    PyObject* py_buffer;

};
