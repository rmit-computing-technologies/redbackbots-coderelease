/**
 * @file RobotDetector.cpp
 * 
 * @author RedbackBots
*/

#include "perception/vision/detector/RobotDetector.hpp"

#include "perception/vision/VisionInfoMiddle.hpp"
#include "perception/vision/VisionInfoOut.hpp"
#include "utils/Logger.hpp"
#include "utils/debug/Assert.hpp"

RobotDetector::RobotDetector(Blackboard* blackboard):
Detector("FieldFeatureDetector")
{
    configure(blackboard);

    llog(INFO) << NDEBUG_LOGSYMB << "RobotDetector loaded" << std::endl;
}

RobotDetector::~RobotDetector() {

}

void RobotDetector::configure(Blackboard* blackboard) {

}

void RobotDetector::resetMiddleInfo(VisionInfoMiddle* info_middle) {
}

void RobotDetector::resetVisionOut(VisionInfoOut* info_out) {
    info_out->robots.clear();
}

void RobotDetector::detect_(const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    // llog(INFO) << __PRETTY_FUNCTION__ << std::endl;

    detect_(CameraInfo::Camera::top, info_in, info_middle, info_out);

    detect_(CameraInfo::Camera::bot, info_in, info_middle, info_out);
}

void RobotDetector::detect_(CameraInfo::Camera whichCamera,
    const VisionInfoIn* info_in, VisionInfoMiddle* info_middle, VisionInfoOut* info_out) {
    
    const CameraInfo& cameraInfo = info_in->cameraInfo[whichCamera];
    RobotObstaclesField& obstaclesField = info_middle->robotsField[whichCamera];
    RobotObstaclesImage& obstaclesImage = info_middle->robotsImage[whichCamera];

    std::vector<FieldFeatureInfo>& features = info_out->features;
    std::vector<RobotVisionInfo>& robots = info_out->robots;

    ASSERT(obstaclesField.obstacles.size() == obstaclesImage.obstacles.size());

    for(size_t i = 0; i < obstaclesField.obstacles.size(); ++i) {
        RobotVisionInfo robot;
        robot.rr =  info_in->kinematicPose.RXYToRobotRelative(obstaclesField.obstacles[i].center, cameraInfo);

        if(obstaclesField.obstacles[i].type == RobotObstaclesField::Type::ownPlayer || obstaclesField.obstacles[i].type == RobotObstaclesField::Type::ownGoalkeeper) {
            robot.type = RobotVisionInfo::Type::rOwnTeam;
        }
        else if(obstaclesField.obstacles[i].type == RobotObstaclesField::Type::opponentPlayer || obstaclesField.obstacles[i].type == RobotObstaclesField::Type::opponentGoalkeeper) {
            robot.type = RobotVisionInfo::Type::rEnemyTeam;
        }
        else {
            robot.type = RobotVisionInfo::Type::rUnknown;
        }

        if(cameraInfo.camera == CameraInfo::Camera::top) {
            robot.cameras = RobotVisionInfo::Cameras::TOP_CAMERA;
            robot.topImageCoords = BBox(Point(obstaclesImage.obstacles[i].left, obstaclesImage.obstacles[i].top), Point(obstaclesImage.obstacles[i].right, obstaclesImage.obstacles[i].bottom));
        }
        else {
            robot.cameras = RobotVisionInfo::Cameras::BOT_CAMERA;
            robot.botImageCoords = BBox(Point(obstaclesImage.obstacles[i].left, obstaclesImage.obstacles[i].top), Point(obstaclesImage.obstacles[i].right, obstaclesImage.obstacles[i].bottom));
        }
        
        robots.emplace_back(robot);
    }

    // llog(INFO) << NDEBUG_LOGSYMB << "robots: " << robots.size() << std::endl;
}