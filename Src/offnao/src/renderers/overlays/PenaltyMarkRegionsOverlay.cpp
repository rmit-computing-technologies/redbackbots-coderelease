/**
 * @file PenaltyMarkRegionsOverlay.hpp
 * 
 * Draw the potential penalty mark regions
 * 
 * @author RedbackBots
 */

#include "renderers/overlays/PenaltyMarkRegionsOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/DebuggerBlackboard.hpp"
#include "blackboard/modules/debuger/VisionDebuggerBlackboard.hpp"
#include "types/vision/PenaltyMarkRegions.hpp"

PenaltyMarkRegionsOverlay::PenaltyMarkRegionsOverlay(CameraInfo::Camera whichCamera) :
    OverlayPainter(),
    whichCamera(whichCamera) 
{
}

void PenaltyMarkRegionsOverlay::_drawOverlay(Blackboard* blackboard) {

    if (blackboard->debugger != nullptr && 
        blackboard->debugger->vision != nullptr) {
            
        // get horizontal scan lines
        PenaltyMarkRegions pmrs = readFrom_debugger(vision, penaltyMarkRegions[whichCamera]);

        // llog(DEBUG) << NDEBUG_LOGSYMB << pmrs.regions.size() << std::endl;

        save();

        setPen_(QColor("blue"), defaultLineWidth);

        for(const Boundaryi boundary : pmrs.regions) {
            // llog(DEBUG) << NDEBUG_LOGSYMB << "boundary.x.min: " << boundary.x.min << std::endl;
            drawRect(QRect(xy2q(boundary.x.min, boundary.y.min), xy2q(boundary.x.max, boundary.y.max)));
        }
    }

    restore();
}