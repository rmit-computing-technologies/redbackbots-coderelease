
/**
 * @file HorizonOverlay.cpp
 * 
 * Draw the horizon
 * 
 * @author RedbackBots
 */

#include "renderers/overlays/HorizonOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/MotionBlackboard.hpp"
#include "perception/kinematics/RobotPose.hpp"

void HorizonOverlay::_drawOverlay(Blackboard* blackboard) {
    std::pair<int, int> horizon = readFrom(motion, pose).getHorizon();
    drawHorizon(horizon);
}

void HorizonOverlay::drawHorizon(std::pair<int, int>& horizon, int width) {
   save();

   int endX = device()->width();
   QPoint p1(0, horizon.first);
   QPoint p2(endX, horizon.second);

   setPen_(QColor("pink"), width);
   drawLine(p1, p2);

   restore();
}

