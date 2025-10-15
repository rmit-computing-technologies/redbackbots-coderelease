/**
 * @file CameraInfo.h
 *
 * Declaration of struct CameraInfo
 *
 * @author <a href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</a>
 * @author <a href="mailto:walter.nistico@uni-dortmund.de">Walter Nistico</a>
 * @author RedbackBots
 */

#pragma once

#include <string>

#include "types/math/Angle.hpp"
#include "types/math/Eigen.hpp"

#include "types/camera/CameraCalibrations.hpp"

/**
 * Information about the camera which provides the images for the robot
 */
class CameraInfo {
public:
    // Enum representing the possible sources of an image.
    enum Camera : int {
        top = 0,
        bot,
        
        // Records number of elements in the enum (based on C++ enum representations)
        NUM_CAMERAS
    };

    CameraInfo() = default;

    void updateFocalLength();
    void onRead() { updateFocalLength(); };

    static std::string enumCameraToString(Camera camera);

    /**
     * Intrinsic camera parameters: axis skew is modelled as 0 (90° perfectly orthogonal XY)
     * and the same has been modelled for focal axis aspect ratio; distortion is considering
     * only 2nd and 4th order coefficients of radial model, which account for about 95% of total.
     */
    float focalLength = 0;
    float focalLengthInv = 0; // (1/focalLength) used to speed up certain calculations
    float focalLenPow2 = 0;
    float focalLengthHeight = 0;
    float focalLengthHeightInv = 0;

    // Public properties
    Camera camera;
    int width;
    int height;
    Angle openingAngleWidth;
    Angle openingAngleHeight;
    Vector2f opticalCenter;

    // cameraCalibration corrections
    CameraCalibrations cameraCalibrations;

    // Properties of pixel ratio
    // float pixelRatioWidth;
    // float pixelRatioHeight;
};
