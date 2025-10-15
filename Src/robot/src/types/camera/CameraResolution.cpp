#include "types/camera/CameraResolution.hpp"

std::string cameraResolutionEnumToString(CameraResolution value) {
    switch (value) {
        case CameraResolution::defaultRes: return "defaultRes";
        case CameraResolution::w320h240: return "w320h240";
        case CameraResolution::w640h480: return "w640h480";
        case CameraResolution::w1280h960: return "w1280h960";
        case CameraResolution::noRequest: return "noRequest";
        default: return "Unknown";
    }
}
