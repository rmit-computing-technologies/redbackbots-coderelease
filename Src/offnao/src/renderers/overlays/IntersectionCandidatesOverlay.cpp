/**
 * @file IntersectionCandidatesOverlay.cpp
 * 
 * Draw the horizon
 * 
 * @author RedbackBots
*/

#include "renderers/overlays/IntersectionCandidatesOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/DebuggerBlackboard.hpp"
#include "blackboard/modules/debuger/VisionDebuggerBlackboard.hpp"
#include "types/vision/IntersectionCandidates.hpp"

IntersectionCandidatesOverlay::IntersectionCandidatesOverlay(CameraInfo::Camera whichCamera) :
    OverlayPainter(),
    whichCamera(whichCamera) 
{
}

void IntersectionCandidatesOverlay::_drawOverlay(Blackboard* blackboard) {
    if (blackboard->debugger != nullptr && 
        blackboard->debugger->vision != nullptr) {

        IntersectionCandidates intersectionCandidates = readFrom_debugger(vision, intersectionCandidates[whichCamera]);

        save();

        setPen_(QColor("purple"), 10);

        for(const IntersectionCandidates::IntersectionCandidate& candidate : intersectionCandidates.intersections){
            drawPoint(v2i2q(candidate.img));
        }

        restore();
    }
}

