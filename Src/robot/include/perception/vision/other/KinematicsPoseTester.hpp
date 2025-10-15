
/**
 * @file KinematicsPoseTester.hpp
 *
 * Test for Kinematics Pose
 * 
 * @author RedbackBots
 */

#pragma once

#include "perception/vision/Detector.hpp"

#include "blackboard/Blackboard.hpp"


class KinematicsPoseTester : public Detector {
public:
    KinematicsPoseTester(Blackboard* blackboard);
    virtual ~KinematicsPoseTester();

    // Resets
    virtual void resetMiddleInfo(VisionInfoMiddle* info_middle);
    virtual void resetVisionOut(VisionInfoOut* info_out);

protected:
    virtual void detect_(const VisionInfoIn* info_in, 
                         VisionInfoMiddle* info_middle,
                         VisionInfoOut* info_out);

private:

    void configure(Blackboard* blackboard);

    /**
     * Run detection for the given camera
     */
    void detect_(CameraInfo::Camera whichCamera,
                 const VisionInfoIn* info_in, 
                 VisionInfoMiddle* info_middle,
                 VisionInfoOut* info_out);
};