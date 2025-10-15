/**
 * @file HorizontalScanLineOverlay.cpp
 * 
 * Draw the horizontal scan lines
 * 
 * @author RedbackBots
 */

#include "renderers/overlays/HorizontalScanLineOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/DebuggerBlackboard.hpp"
#include "blackboard/modules/debuger/VisionDebuggerBlackboard.hpp"
#include "types/vision/ColorScanLineRegions.hpp"

HorizontalScanLineOverlay::HorizontalScanLineOverlay(CameraInfo::Camera whichCamera) :
    OverlayPainter(),
    whichCamera(whichCamera) 
{
}

void HorizontalScanLineOverlay::_drawOverlay(Blackboard* blackboard) {

    if (blackboard->debugger != nullptr && 
        blackboard->debugger->vision != nullptr) {
            
        // get horizontal scan lines
        ColorScanLineRegionsHorizontal hsl = readFrom_debugger(vision, colorScanLineRegionsHorizontal[whichCamera]);

        save();

        for(const ColorScanLineRegionsHorizontal::ScanLine& line : hsl.scanLines) {
            for(const ScanLineRegion& region : line.regions){
                if(region.color == ScanLineRegion::Color::white){
                    setPen_(QColor("red"), defaultScanLineWidth);
                }
                else if(region.color == ScanLineRegion::Color::field){
                    setPen_(QColor("green"), defaultScanLineWidth);
                }
                else if(region.color == ScanLineRegion::Color::unset || region.color == ScanLineRegion::Color::none) {
                    setPen_(QColor("gray"), defaultScanLineWidth);
                }
                else{
                    continue;
                }
                drawLine(xy2q(region.range.from, line.y), xy2q(region.range.to, line.y));
            }
        }
    }

    restore();
}