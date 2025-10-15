/**
 * @file RefereeKeypointsOverlay.cpp
 * 
 * Draw the referee keypoints and current gesture
 * 
 * @author RedbackBots
*/

#include "renderers/overlays/RefereeKeypointsOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/DebuggerBlackboard.hpp"
#include "blackboard/modules/debuger/VisionDebuggerBlackboard.hpp"
#include "types/vision/RefereeKeypoints.hpp"

RefereeKeypointsOverlay::RefereeKeypointsOverlay(CameraInfo::Camera whichCamera) :
    OverlayPainter(),
    whichCamera(whichCamera) 
{
}

void RefereeKeypointsOverlay::_drawOverlay(Blackboard* blackboard) {

    if (blackboard->debugger != nullptr && 
        blackboard->debugger->vision != nullptr) {
            
        RefereeKeypoints rk = readFrom_debugger(vision, refereeKeypoints);
        RefereeGesture rg = readFrom(vision, refereeGesture);

        save();

        setPen_(QColor("red"), 6);

        for(const RefereeKeypoints::Point& point : rk.points){
            drawPoint(v2f2q(point.position));
        }

        QFont font = QFont();
        font.setPixelSize(48);
        setFont(font);
        const QRect rectangle = QRect(0, 0, 320, 50);
        drawText(rectangle, 0, gestureToString(rg.gesture).c_str());
    }

    restore();
}