/**
 * @file VerticalScanLineOverlay.cpp
 * 
 * Draw the vertical scan lines
 * 
 * @author RedbackBots
 */

#include "renderers/overlays/VerticalScanLineOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/DebuggerBlackboard.hpp"
#include "blackboard/modules/debuger/VisionDebuggerBlackboard.hpp"
#include "types/vision/ColorScanLineRegions.hpp"

VerticalScanLineOverlay::VerticalScanLineOverlay(CameraInfo::Camera whichCamera) :
    OverlayPainter(),
    whichCamera(whichCamera) 
{
}

void VerticalScanLineOverlay::_drawOverlay(Blackboard* blackboard) {

    if (blackboard->debugger != nullptr && 
        blackboard->debugger->vision != nullptr) {
            
        // get horizontal scan lines
        ColorScanLineRegionsVerticalClipped vsl = readFrom_debugger(vision, colorScanLineRegionsVerticalClipped[whichCamera]);

        save();

        for(const ColorScanLineRegionsVerticalClipped::ScanLine& line : vsl.scanLines) {
            for(const ScanLineRegion& region : line.regions){
                if(region.color == ScanLineRegion::Color::white){
                    setPen_(QColor("red"), defaultScanLineWidth);
                }
                else if(region.color == ScanLineRegion::Color::field){
                    setPen_(QColor("green"), defaultScanLineWidth);
                }
                else if(region.color == ScanLineRegion::Color::unset || region.color == ScanLineRegion::Color::none){
                    setPen_(QColor("gray"), defaultScanLineWidth);
                }
                else{
                    continue;
                }
                drawLine(xy2q(line.x, region.range.from), xy2q(line.x, region.range.to));
            }
        }
    }

    restore();
}