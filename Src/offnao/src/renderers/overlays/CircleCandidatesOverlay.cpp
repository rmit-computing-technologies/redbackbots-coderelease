/**
 * @file VerticalScanLineOverlay.cpp
 * 
 * Draw the vertical scan lines
 * 
 * @author RedbackBots
 */

#include "renderers/overlays/CircleCandidatesOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/DebuggerBlackboard.hpp"
#include "blackboard/modules/debuger/VisionDebuggerBlackboard.hpp"
#include "types/vision/LineSpots.hpp"
#include "types/vision/CircleCandidates.hpp"

CircleCandidatesOverlay::CircleCandidatesOverlay(CameraInfo::Camera whichCamera, bool groupedCandidates) :
    OverlayPainter(),
    whichCamera(whichCamera),
    groupedCandidates(groupedCandidates)
{}

void CircleCandidatesOverlay::drawCross(QPoint point, int size){
    int x1 = point.x() - size;
    int y1 = point.y() + size;
    int x2 = point.x() + size;
    int y2 = point.y() - size;

    drawLine(x1, y1, x2, y2);
    drawLine(x1, y2, x2, y1);
}

void CircleCandidatesOverlay::_drawOverlay(Blackboard* blackboard) {

    if (blackboard->debugger != nullptr && 
        blackboard->debugger->vision != nullptr) {
        
        CircleCandidates c = readFrom_debugger(vision, circleCandidates[whichCamera]);
        
        save();
        
        std::vector<QColor> colors = {QColor("cyan"), QColor("orange"), QColor("green"), QColor("pink"), QColor("yellow")};
        int colorIndex = 0;  // Start with the first color
        setPen_(QColor("cyan"), 2);
        for (const CircleCandidate& circleCandidate: c.candidates){
            QPoint lastPoint;  
            bool hasLastPoint = false;
            if (groupedCandidates){
                setPen_(colors[colorIndex % colors.size()], 2); // Cycle through colors
            }
            for (const Vector2f& spot: circleCandidate.imageSpots) {
                QPoint currentPoint(spot.x(), spot.y());
                drawCross(currentPoint, 6);

                if (groupedCandidates && hasLastPoint) {
                    drawLine(lastPoint.x(), lastPoint.y(), currentPoint.x(), currentPoint.y());
                }

                lastPoint = currentPoint;
                hasLastPoint = true;
            }
            colorIndex++;  // Move to the next color
        }
    }

    restore();
}