
/**
 * @file HorizonOverlay.cpp
 * 
 * Draw the horizon
 * 
 * @author RedbackBots
 */

#include "renderers/overlays/BallOverlay.hpp"

#include "blackboard/Blackboard.hpp"
#include "blackboard/modules/VisionBlackboard.hpp"
#include "types/BallInfo.hpp"

BallOverlay::BallOverlay(CameraInfo::Camera whichCamera) :
    OverlayPainter(),
    whichCamera(whichCamera) 
{
}

void BallOverlay::_drawOverlay(Blackboard* blackboard) {

    const std::vector<BallInfo> balls = readFrom(vision, balls);
    
    //llog(INFO) << balls.size() << std::endl;

    save();

    for(const BallInfo& ball : balls){

        if ((!ball.topCamera && whichCamera == CameraInfo::Camera::top) || (ball.topCamera && whichCamera == CameraInfo::Camera::bot)) {
            continue;
        }

        //llog(INFO) << ball.imageCoords << std::endl;
        //llog(INFO) << ball.status << std::endl;
        if (ball.status == BallInfo::seen) {
            setPen_(QColor("green"), 3);//defaultBallWidth);
            QPoint centre = p2q(ball.imageCoords);
            int radius = ball.radius; 
            drawEllipse(centre, radius, radius);
            
            setPen_(QColor("green"), 10);//defaultBallWidth);
            drawPoint(v2i2q(ball.imageCoords));
        }
        else if (ball.status == BallInfo::guessed) {
            setPen_(QColor("yellow"), 3);//defaultBallWidth);
            QPoint centre = p2q(ball.imageCoords);
            int radius = ball.radius; 
            drawEllipse(centre, radius, radius);

            setPen_(QColor("yellow"), 10);//defaultBallWidth);
            drawPoint(v2i2q(ball.imageCoords));
        }
    }

    restore();
}

