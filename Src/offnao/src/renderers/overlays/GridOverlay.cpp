
/**
 * @file GridOverlay.cpp
 * 
 * Draw grid over the image
 * 
 * @author RedbackBots
 */

#include "renderers/overlays/GridOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/MotionBlackboard.hpp"
#include "perception/kinematics/RobotPose.hpp"

GridOverlay::GridOverlay(CameraInfo::Camera whichCamera) :
    OverlayPainter(),
    whichCamera(whichCamera)
{
}

void GridOverlay::_drawOverlay(Blackboard* blackboard) {
    
    setPen_(QColor("cyan"), width);

    if (whichCamera == CameraInfo::Camera::top){
        // TODO: Read the height and width from blackboard 
        for (int y = 0; y < 480; y += 50){
            drawLine(0,y,640,y);
        }
    }
    else {
        setPen_(QColor("grey"), width/2);
        for (int y = 0; y < 240; y += 5){
            drawLine(0,y,320,y);
        } 
        setPen_(QColor("cyan"), width);
        for (int y = 0; y < 240; y += 50){
            drawLine(0,y,320,y);
        } 
    }

}
