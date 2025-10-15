
/**
 * @file HorizonOverlay.cpp
 * 
 * Draw the horizon
 * 
 * @author RedbackBots
 */

#include "renderers/overlays/BallSpotsOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/DebuggerBlackboard.hpp"
#include "blackboard/modules/debuger/VisionDebuggerBlackboard.hpp"
#include "types/vision/BallSpots.hpp"

BallSpotsOverlay::BallSpotsOverlay(CameraInfo::Camera whichCamera) :
    OverlayPainter(),
    whichCamera(whichCamera) 
{
}

void BallSpotsOverlay::_drawOverlay(Blackboard* blackboard) {
    if (blackboard->debugger != nullptr && 
        blackboard->debugger->vision != nullptr) {

        BallSpots ballSpots = readFrom_debugger(vision, ballSpots[whichCamera]);

        save();

        setPen_(QColor("orange"), 10);

        for(const Vector2i& spot : ballSpots.ballSpots){
            drawPoint(v2i2q(spot));
        }

        restore();
    }
}

