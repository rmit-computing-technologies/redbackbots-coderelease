/**
 * @file RobotDetector.hpp
 * 
 * @author RedbackBots
 */

#pragma once

#include "perception/vision/Detector.hpp"
#include "blackboard/Blackboard.hpp"

#include "types/RobotVisionInfo.hpp"
#include "types/vision/RobotObstaclesField.hpp"
#include "types/vision/RobotObstaclesImage.hpp"
#include "types/camera/CameraInfo.hpp"
#include "types/math/Eigen.hpp"
#include "types/geometry/RRCoord.hpp"

class RobotDetector : public Detector {
public:

	RobotDetector(Blackboard* blackboard);
  	virtual ~RobotDetector();

	// Resets middle info
	virtual void resetMiddleInfo(VisionInfoMiddle* info_middle);
	virtual void resetVisionOut(VisionInfoOut* info_out);

protected:
  	virtual void detect_(const VisionInfoIn* info_in, 
                      	VisionInfoMiddle* info_middle,
                      	VisionInfoOut* info_out);
private:

	void configure(Blackboard* blackboard);

	void detect_(CameraInfo::Camera whichCamera,
                const VisionInfoIn* info_in, 
                VisionInfoMiddle* info_middle,
                VisionInfoOut* info_out);
};