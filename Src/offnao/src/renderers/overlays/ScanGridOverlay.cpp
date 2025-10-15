
/**
 * @file HorizonOverlay.cpp
 * 
 * Draw the horizon
 * 
 * @author RedbackBots
 */

#include "renderers/overlays/ScanGridOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/DebuggerBlackboard.hpp"
#include "blackboard/modules/debuger/VisionDebuggerBlackboard.hpp"
#include "types/vision/ScanGrid.hpp"

ScanGridOverlay::ScanGridOverlay(CameraInfo::Camera whichCamera) :
    OverlayPainter(),
    whichCamera(whichCamera) 
{
}

void ScanGridOverlay::_drawOverlay(Blackboard* blackboard) {

    if (blackboard->debugger != nullptr && 
        blackboard->debugger->vision != nullptr) {

        // get scan grid
        ScanGrid sg = readFrom_debugger(vision, scanGrid[whichCamera]);
        //llog(INFO) << "sg vertical line size: " << sg.verticalLines.size() << std::endl;
        //llog(INFO) << "sg horizontal line size: " << sg.lowResHorizontalLines.size() << std::endl;

        save();

        setPen_(QColor("blue"), defaultScanGridWidth);

        for(const ScanGrid::ScanGridHorizontalLine& hLine : sg.lowResHorizontalLines) {
          for(const ScanGrid::ScanGridLine& vLine : sg.verticalLines) {
            if (vLine.x <= hLine.left || vLine.x > hLine.right || hLine.y <= vLine.yMin || hLine.y > vLine.yMax) {
              continue;
            }
            drawLine(xy2q(vLine.x, hLine.y - 1), xy2q(vLine.x, hLine.y));
          }
        }

        // old scan lines
        // for(const ScanGrid::ScanGridLine& line : sg.verticalLines) {
        //   for(auto i = sg.fullResY.begin() + line.yMaxIndex; i != sg.fullResY.end(); ++i) {
        //     drawLine(xy2q(line.x, *i - 1), xy2q(line.x, *i));
        //   }
        // }

        restore();
    }
}

