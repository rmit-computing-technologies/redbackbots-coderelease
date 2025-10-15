/**
 * @file VerticalScanLineOverlay.cpp
 * 
 * Draw the vertical scan lines
 * 
 * @author RedbackBots
 */

#include "renderers/overlays/IsWhiteSpotsOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/DebuggerBlackboard.hpp"
#include "blackboard/modules/debuger/VisionDebuggerBlackboard.hpp"
#include "types/vision/IsWhiteSpots.hpp"

IsWhiteSpotsOverlay::IsWhiteSpotsOverlay(CameraInfo::Camera whichCamera) :
    OverlayPainter(),
    whichCamera(whichCamera)
{}

void IsWhiteSpotsOverlay::drawCross(QPoint point, int size){

    int x1 = std::max(0, point.x() - size);
    int y1 = std::max(0, point.y() + size);
    int x2 = std::max(0, point.x() + size);
    int y2 = std::max(0, point.y() - size);

    drawLine(x1, y1, x2, y2);
    drawLine(x1, y2, x2, y1);
}


void IsWhiteSpotsOverlay::_drawOverlay(Blackboard* blackboard) {

    if (blackboard->debugger != nullptr && 
        blackboard->debugger->vision != nullptr) {
        
        IsWhiteSpots isWhiteSpots;
        isWhiteSpots = readFrom_debugger(vision, isWhiteSpots[whichCamera]);

        save();
        setPen_(QColor("Orange"), defaultLineSpotsWidth);


        for (Comparison comparison : isWhiteSpots.comparisons){
            // std::cout << "comparison.type:" << comparison.type << std::endl;

            // Choose color based on type
            QColor color;
            switch (comparison.type) {
                case 1:
                    color = QColor("red");
                    break;
                case 2:
                    color = QColor("white");
                    break;
                case 3:
                    color = QColor("blue");
                    break;
                case 4:
                    color = QColor("yellow");
                    break;
                default:
                    color = QColor("orange"); // fallback color
            }

            setPen_(color, defaultLineSpotsWidth);


            drawCross(QPoint(comparison.referencePoint.image.x(), comparison.referencePoint.image.y()), 2);
            drawLine(QPoint(comparison.pointInImage.image.x(), comparison.pointInImage.image.y()),
                    QPoint(comparison.referencePoint.image.x(), comparison.referencePoint.image.y()));
        }
         
    }

    restore();
}