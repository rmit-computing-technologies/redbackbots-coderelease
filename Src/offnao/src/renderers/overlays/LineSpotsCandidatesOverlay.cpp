/**
 * @file VerticalScanLineOverlay.cpp
 * 
 * Draw the vertical scan lines
 * 
 * @author RedbackBots
 */

#include "renderers/overlays/LineSpotsCandidatesOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/DebuggerBlackboard.hpp"
#include "blackboard/modules/debuger/VisionDebuggerBlackboard.hpp"
#include "types/vision/LineSpots.hpp"
#include "types/vision/Candidates.hpp"

LineSpotsCandidatesOverlay::LineSpotsCandidatesOverlay(CameraInfo::Camera whichCamera, int maxX, int maxY, Type type, bool groupedCandidates) :
    OverlayPainter(),
    whichCamera(whichCamera),
    maxX(maxX),
    maxY(maxY),
    type(type),
    groupedCandidates(groupedCandidates)
{
    switch(type) {
        case Type::Final:
            horizontalColour = QColor("#ff0000"); // Red
            verticalColour = QColor("#0000ff"); // Blue
            lineThickness = 6;
            break;
        case Type::Before:
            horizontalColour = QColor("#8000ced1"); // darkturquoise
            verticalColour = QColor("#8000ced1");  // darkturquoise
            lineThickness = defaultLineSpotsWidth;
            break;
        case Type::After:
            horizontalColour = QColor("#80ffff00"); // Yellow
            verticalColour = QColor("#80ffff00"); // Yellow
            lineThickness = defaultLineSpotsWidth;
            break;
    }
}

void LineSpotsCandidatesOverlay::drawCross(QPoint point, int size){
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

void LineSpotsCandidatesOverlay::drawPlus(QPoint point, int size){
    const int MAX_X = static_cast<int>(maxX);
    const int MAX_Y = static_cast<int>(maxY);
    if (point.x() > MAX_X || point.y() > MAX_Y || point.x() < 0 || point.y() < 0){
        std::cout << "point outside of camera bounds" << std::endl;
        return;
    }

    int x1 = std::max(0, std::min(MAX_X, point.x() - size));
    int x2 = std::max(0, std::min(MAX_X, point.x() + size));
    int y1 = std::max(0, std::min(MAX_Y, point.y() + size));
    int y2 = std::max(0, std::min(MAX_Y, point.y() - size));

    drawLine(x1, point.y(), x2, point.y());
    drawLine(point.x(), y1, point.x(), y2);
}

void LineSpotsCandidatesOverlay::_drawOverlay(Blackboard* blackboard) {

    if (blackboard->debugger != nullptr && 
        blackboard->debugger->vision != nullptr) {
        
        Candidates c;
         switch(type) {
            case Type::Final:
                c = readFrom_debugger(vision, candidates[whichCamera]);
                // std::cout << "Final: " << c.horizontalCandidates.size() << " , " << c.verticalCandidates.size() << std::endl;
                break;
            case Type::Before:
                c = readFrom_debugger(vision, candidatesBefore[whichCamera]);
                // std::cout << "Before: " << c.horizontalCandidates.size() << " , " << c.verticalCandidates.size() << std::endl;
                break;
            case Type::After:
                c = readFrom_debugger(vision, candidatesAfter[whichCamera]);
                // std::cout << "After: " << c.horizontalCandidates.size() << " , " << c.verticalCandidates.size() << std::endl;
                break;
        }

        save();
        std::vector<QColor> horizontalColours = {
            QColor("cyan"),
            QColor("orange"),
            QColor("green"),
            QColor("pink"),
            QColor("yellow")
        };
        int colorIndex = 0;  // Start with the first color
        setPen_(horizontalColour, lineThickness);
        for (const Candidate& candidate: c.horizontalCandidates){
            QPoint lastPoint;  
            bool hasLastPoint = false;
            if (groupedCandidates){
                setPen_(horizontalColours[colorIndex % horizontalColours.size()], 2); // Cycle through colors
            }
            for (const Spot* spot: candidate.spots) {
                QPoint currentPoint(spot->image.x(), spot->image.y());
                if (type == Type::Final){
                    drawCross(currentPoint, 6);
                } else {
                    drawPlus(currentPoint, 6);
                }

                if (groupedCandidates && hasLastPoint) {
                    drawLine(lastPoint.x(), lastPoint.y(), currentPoint.x(), currentPoint.y());
                }

                lastPoint = currentPoint;
                hasLastPoint = true;
            }
            colorIndex++;  // Move to the next color
        }

        std::vector<QColor> verticalColours = {
            QColor("navy"),       // very dark blue
            QColor("maroon"),     // deep red-brown
            QColor("teal"),       // dark cyan-green
            QColor("purple"),     // saturated violet
            QColor("olive")       // muted yellow-green
        };
        colorIndex = 0;  // Start with the first color
        setPen_(verticalColour, lineThickness);
        for (const Candidate& candidate: c.verticalCandidates){
            QPoint lastPoint;  
            bool hasLastPoint = false;
            if (groupedCandidates){
                setPen_(verticalColours[colorIndex % verticalColours.size()], 2); // Cycle through colors
            }
            for (const Spot* spot: candidate.spots) {
                QPoint currentPoint(spot->image.x(), spot->image.y());
                if (type == Type::Final){
                    drawCross(v2f2q(spot->image), 6);
                } else {
                    drawPlus(v2f2q(spot->image), 6);
                }

                if (groupedCandidates && hasLastPoint) {
                    drawLine(lastPoint.x(), lastPoint.y(), currentPoint.x(), currentPoint.y());
                }

                lastPoint = currentPoint;
                hasLastPoint = true;
            }
            colorIndex++; 
        }


    }

    restore();
}