/**
 * @file CameraImageRender.hpp
 * 
 * Renders a camera image as a QLabel.
 * Optionally can render a collection of overlays onto the image
 * 
 * @author RedbackBots
 * 
 */

#pragma once

#include "types/camera/CameraImage.hpp"
#include "types/camera/CameraInfo.hpp"

#include "renderers/OverlayPainter.hpp"

#include <QLabel>
#include <QPixmap>

// Forward Declarations
class Blackboard;


class CameraImageRender : public QLabel {
Q_OBJECT
public:
    static constexpr int NO_IMAGE_DIM = 100;

    CameraImageRender(int cols, int rows, CameraInfo::Camera whichCamera);
    virtual ~CameraImageRender();

    // Add an overlay to draw on this camera
    void addOverlay(OverlayPainter* overlay);

    // Provide the latest camera image -> doesn't redraw
    void setCameraImage(CameraImage& cameraImage);

    // Provide the latest blackboard data
    void setBlackboard(Blackboard *blackboard);

    // Update the camera image render with the current info/settings
    void redraw();

private:
    // Final dimension of the image
    int rows;
    int cols;

    // Which camera
    CameraInfo::Camera whichCamera;

    // Pixmap for generating the image draw
    QPixmap imagePixmap;

    // Blackboard ref
    Blackboard *blackboard;

    // Latest camera image
    CameraImage cameraImage;

    // Overlays to draw on this camera
    std::vector<OverlayPainter*> overlays;

    // Draw the image pixels
    void drawImage(QImage* qImage);

    // Draw overlays for image
    void drawOverlays(QPixmap *pixmap);
};
