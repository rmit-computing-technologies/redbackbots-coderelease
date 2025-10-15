#include "perception/vision/camera/CombinedCamera.hpp"
#include "utils/Timer.hpp"
#include "utils/Logger.hpp"

NaoCameraProvider* CombinedCamera::top_camera_ = NULL;
NaoCameraProvider* CombinedCamera::bot_camera_ = NULL;

CombinedCamera::CombinedCamera() {
}

CombinedCamera::~CombinedCamera(){
}

NaoCameraProvider* CombinedCamera::getCamera() {
    return top_camera_;
}

void CombinedCamera::getFrameTop() {
    const uint8_t* image = top_camera_->takeImage();
}

void CombinedCamera::getFrameBottom() {
    bot_camera_->takeImage();
}

NaoCameraProvider* CombinedCamera::getCameraTop() {
    return top_camera_;
}

NaoCameraProvider* CombinedCamera::getCameraBot() {
    return bot_camera_;
}

void CombinedCamera::setCameraTop(NaoCameraProvider* camera) {
    top_camera_ = camera;
}

void CombinedCamera::setCameraBot(NaoCameraProvider* camera) {
    bot_camera_ = camera;
}

CombinedCameraSettings CombinedCamera::getCameraSettings(){
    NaoCameraProvider* top = (NaoCameraProvider*)(top_camera_);
    NaoCameraProvider* bot = (NaoCameraProvider*)(bot_camera_);
    
    CombinedCameraSettings settings;
    settings.top_camera_settings = top->cameraSettings;
    settings.bot_camera_settings = bot->cameraSettings;
    return settings;
}
