/**
 * @file RobotOverlay.cpp
 * 
 * Draw the robot bounding boxes
 * 
 * @author RedbackBots
 */

#include "renderers/overlays/RobotOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/DebuggerBlackboard.hpp"
#include "blackboard/modules/debuger/VisionDebuggerBlackboard.hpp"
#include "types/geometry/Point.hpp"
#include "types/vision/RobotObstaclesImage.hpp"

RobotOverlay::RobotOverlay(CameraInfo::Camera whichCamera) :
    OverlayPainter(),
    whichCamera(whichCamera)
{
}

void RobotOverlay::_drawOverlay(Blackboard* blackboard) {

    if (blackboard->debugger != nullptr && 
        blackboard->debugger->vision != nullptr) {
            
        RobotObstaclesImage robots = readFrom_debugger(vision, robotsImage[whichCamera]);

        save();

        setPen_(QColor("purple"), 3);
        
        for (const RobotObstaclesImage::Obstacle& robot : robots.obstacles) {
            //const QRect box = QRect(robot.top, robot.left, robot.right - robot.left, robot.bottom - robot.top);
            //drawRect(box);
            
            drawLine(xy2q(robot.left, robot.top), xy2q(robot.left, robot.bottom));
            drawLine(xy2q(robot.left, robot.top), xy2q(robot.right, robot.top));
            drawLine(xy2q(robot.left, robot.bottom), xy2q(robot.right, robot.bottom));
            drawLine(xy2q(robot.right, robot.top), xy2q(robot.right, robot.bottom));
        }

    restore();
    }
}