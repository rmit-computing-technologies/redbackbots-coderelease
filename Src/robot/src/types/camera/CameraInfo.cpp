/**
 * @file CameraInfo.cpp
 * Implementation of struct CameraInfo
 */

#include "types/camera/CameraInfo.hpp"
#include "utils/math/basic_maths.hpp"
#include "utils/Logger.hpp"

std::string CameraInfo::enumCameraToString(Camera camera) {
    std::string str = "unknown";
    if (camera == Camera::top) {
        str = "camera.top";
    } else if (camera == Camera::bot) {
        str = "camera.bot";
    }
    return str;
}

void CameraInfo::updateFocalLength() {
    /**
     *  TW Note (so it doesn't get changed again).
     *  Opening angles are of the Angle data structure which store values in radians.
     *  NaoCameraProvider.cpp correctly sets the opening angles as radians
     *    in the readCameraIntrinsics method as reading of configuration / options
     *    is the correct place to ensure data types are configured in the right format/unit.
     */ 
    focalLength = width / (2.f * std::tan(openingAngleWidth / 2.f));
    //llog(DEBUG) << "CameraInfo focal length: " << focalLength << std::endl;
    focalLengthInv = 1.f / focalLength;
    focalLenPow2 = SQUARE(focalLength);

    focalLengthHeight = height / (2.f * std::tan(openingAngleHeight / 2.f));
    //llog(DEBUG) << "CameraInfo focal length height: " << focalLengthHeight << std::endl;
    focalLengthHeightInv = 1.f / focalLengthHeight;

    // pixelRatioHeight = std::tan(openingAngleHeight / 2.f) / height;
    // pixelRatioWidth = std::tan(openingAngleWidth / 2.f) / width;
}
