/**
 * @file VerticalScanLineOverlay.cpp
 * 
 * Draw the vertical scan lines
 * 
 * @author RedbackBots
 */

#include "renderers/overlays/PlaneSpotsOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/DebuggerBlackboard.hpp"
#include "blackboard/modules/debuger/VisionDebuggerBlackboard.hpp"
#include "types/vision/PlaneSpots.hpp"

PlaneSpotsOverlay::PlaneSpotsOverlay(CameraInfo::Camera whichCamera, int maxX, int maxY) :
    OverlayPainter(),
    whichCamera(whichCamera),
    maxX(maxX),
    maxY(maxY)
{
}

void PlaneSpotsOverlay::drawCross(QPoint point, int size){
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

void PlaneSpotsOverlay::_drawOverlay(Blackboard* blackboard) {

    if (blackboard->debugger != nullptr && 
        blackboard->debugger->vision != nullptr) {
            
        PlaneSpots planeSpots = readFrom_debugger(vision, planeSpots[whichCamera]);

        save();

        setPen_(QColor("red"), 3);
        for (const Spot* spot: planeSpots.spots) {
            drawCross(v2f2q(spot->image), 3);
        }

    }

    restore();
}