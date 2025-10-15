/**
 * @file RefereeGesture.hpp
 *
 * Very simple representation of the referee gesture.
 *
 * @author <a href="mailto:aylu@uni-bremen.de">Ayleen Lührsen</a>
 * @author RedbackBots
*/

#pragma once

class RefereeGesture {
public:
    enum Gesture {
        none,
        kickInBlue,
        kickInRed,
        goalKickBlue,
        goalKickRed,
        cornerKickBlue,
        cornerKickRed,
        goalBlue,
        goalRed,
        pushingFreeKickBlue,
        pushingFreeKickRed,
        fullTime,
        substitution,
        standbyToReady,
        count
    };

    Gesture gesture = Gesture::none;
};