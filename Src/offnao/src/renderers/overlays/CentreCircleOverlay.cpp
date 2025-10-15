/**
 * @file CentreCircleOverlay.cpp
 * 
 * Draw the centre circle
 * 
 * @author RedbackBots
 */

#include "renderers/overlays/CentreCircleOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/DebuggerBlackboard.hpp"
#include "blackboard/modules/debuger/VisionDebuggerBlackboard.hpp"
#include "types/vision/CentreCircle.hpp"

CentreCircleOverlay::CentreCircleOverlay(CameraInfo::Camera whichCamera, int maxX, int maxY) :
    OverlayPainter(),
    whichCamera(whichCamera),
    maxX(maxX),
    maxY(maxY)
{
}

void CentreCircleOverlay::drawCross(QPoint point, int size){
    const int MAX_X = static_cast<int>(maxX);
    const int MAX_Y = static_cast<int>(maxY);
    if (point.x() > MAX_X || point.y() > MAX_Y || point.x() < 0 || point.y() < 0){
        std::cout << "point outside of camera bounds" << std::endl;
        return;
    }

    int x1 = std::max(0, std::min(MAX_X, point.x() - size));
    int y1 = std::max(0, std::min(MAX_Y, point.y() + size));
    int x2 = std::max(0, std::min(MAX_X, point.x() + size));
    int y2 = std::max(0, std::min(MAX_Y, point.y() - size));

    drawLine(x1, y1, x2, y2);
    drawLine(x1, y2, x2, y1);
}
 
void CentreCircleOverlay::_drawOverlay(Blackboard* blackboard) {

    if (blackboard->debugger != nullptr && 
    blackboard->debugger->vision != nullptr) {
                
        // get centre circle
        CentreCircle cc = readFrom_debugger(vision, centreCircle[whichCamera]);

        save();

        setPen_(QColor("purple"), defaultWidth);

        if(cc.wasSeen) {
            drawCross(v2i2q(cc.image), 6);
        }
    }

    restore();
}