#pragma once

#include "types/camera/CombinedCameraSettings.hpp"
#include "perception/vision/camera/NaoCameraProvider.hpp"

/**
 * CombinedCamera
 * 
 * CombinedCamera abstracts the top and bottom camera
 *
 * The cameras must be set at some point during initialization:
 *
 *     CombinedCamera::topCamera = new SomeCamera(...);
 *     CombinedCamera::botCamera = new SomeCamera(...);
 *
 */
class CombinedCamera {
public:
    /**
     * CombinedCamera()
     */
    CombinedCamera();
    ~CombinedCamera();

    NaoCameraProvider* getCamera();

    /**
     * getFrame()
     *
     * Return a CombinedFrame object containing the current image
     * from both the top and bottom camera
     */
    void getFrameTop();
    void getFrameBottom();
    static NaoCameraProvider* getCameraTop();
    static NaoCameraProvider* getCameraBot();
    static void setCameraTop(NaoCameraProvider* camera);
    static void setCameraBot(NaoCameraProvider* camera);

    /**
     * getCameraSettings()
     *
     * Return a CombinedCameraSettings object containing the current settings
     * of the top and the bottom camera
     */
    CombinedCameraSettings getCameraSettings();

private:
    // Top and Bot camera are currently static so OffNao transmitter can receive commands
    //     from OffNao to set camera commands.
    // TODO (TW): Refactor this hack to use Blackboard now with new camera settings setup
    static NaoCameraProvider *top_camera_;
    static NaoCameraProvider *bot_camera_;
};

