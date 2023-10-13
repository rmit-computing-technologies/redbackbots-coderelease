/*
* BehaviourSharedData.hpp
*
*  Created on: 15/05/2014
*      Author: osushkov
*/

#pragma once

class BehaviourSharedData {
public:
    BehaviourSharedData();

    BehaviourSharedData(
        int secondsSinceLastKick,
        int role,
        bool playingBall,
        bool needAssistance,
        bool isAssisting,
        bool isKickedOff,
        float walkingToX,
        float walkingToY,
        float walkingToH,
        bool kickNotification
    ):
        secondsSinceLastKick(secondsSinceLastKick),
        role(role),
        playingBall(playingBall),
        needAssistance(needAssistance),
        isAssisting(isAssisting),
        isKickedOff(isKickedOff),
        walkingToX(walkingToX),
        walkingToY(walkingToY),
        walkingToH(walkingToH),
        kickNotification(kickNotification)
    {};

    // Number of seconds since the robot last kicked the ball
    int secondsSinceLastKick;

    // The encoded enum value for the robots role
    int role = -1;

    // Whether the robot is playing the ball or not
    bool playingBall;

    // Whether the robot needs assistance
    bool needAssistance;

    // Whether the robot is assisting
    bool isAssisting;

    bool isKickedOff;

    float walkingToX; // x of where a robot is walking to
    float walkingToY; // y of where a robot is walking to
    float walkingToH; // theta of where a robot is walking to

    // Notification to other robots that robot is about to kick the ball.
    bool kickNotification;

    bool sanityCheck();
};
