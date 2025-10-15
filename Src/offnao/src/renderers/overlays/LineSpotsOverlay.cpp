/**
 * @file VerticalScanLineOverlay.cpp
 * 
 * Draw the vertical scan lines
 * 
 * @author RedbackBots
 */

#include "renderers/overlays/LineSpotsOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/DebuggerBlackboard.hpp"
#include "blackboard/modules/debuger/VisionDebuggerBlackboard.hpp"
#include "types/vision/LineSpots.hpp"
#include "types/vision/Candidates.hpp"

LineSpotsOverlay::LineSpotsOverlay(CameraInfo::Camera whichCamera, int maxX, int maxY) :
    OverlayPainter(),
    whichCamera(whichCamera),
    maxX(maxX),
    maxY(maxY)
{
}

void LineSpotsOverlay::_drawOverlay(Blackboard* blackboard) {

    if (blackboard->debugger != nullptr && 
        blackboard->debugger->vision != nullptr) {
            
        LineSpots ls = readFrom_debugger(vision, lineSpots[whichCamera]);

        // std::cout << "LineSpots " << ls.lines.size() << std::endl;
        save();

        setPen_(QColor("blue"), 3);
        
        for (const LineSpots::Line& line : ls.lines) {
            if (line.belongsToCircle) {
                setPen_(QColor("cyan"), 3);
                drawLine(xy2q(line.firstImg.x(), line.firstImg.y()), xy2q(line.lastImg.x(), line.lastImg.y()));
            }
            else {
                setPen_(QColor("blue"), 3);
                drawLine(xy2q(line.firstImg.x(), line.firstImg.y()), xy2q(line.lastImg.x(), line.lastImg.y()));
            }
        }
    }

    restore();
}