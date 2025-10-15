/**
 * @file CameraResolution.hpp
 *
 * This file implements a representation of a camera resolution.
 *
 * @author Dana Jenett, Alexis Tsogias, Felix Thielke
 * @author RedbackBots
 */

#pragma once

#include <string>

enum CameraResolution {
    defaultRes,   /**< Use resolutions as specified in the config file. */
    w320h240,     /**< 160x120 in YCbCr222 */
    w640h480,     /**< 320x240 in YCbCr222 */
    w1280h960,    /**< 640x480 in YCbCr222 */
    noRequest,    /**< Keep the resolution as it is. */
};

std::string cameraResolutionEnumToString(CameraResolution value);
