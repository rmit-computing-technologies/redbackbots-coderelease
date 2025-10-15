/**
 * ActionCommand.hpp
 * Description: Commands which are accepted by the Locomotion Module
 * This is the new interface between the Behaviour and Locomotion
 * Body are for the walks and special actions which use the body
 * Head are for the head yaw and pitch
 * LED are for the ear, face, chest and foot LEDs
 * Tidied up from 2009 code
 */

#pragma once

#include <iostream>
#include <stdint.h>
#include <boost/python.hpp>

#include "utils/body.hpp"

/* Remember to update your constants in python/wrappers/ActionCommand */

namespace ActionCommand {

/**
 * Command for controlling the body
 * Note: Some ActionType Commands WILL disable the head
 */
   struct Body {
      // Predefined actions. These take precedence over walk parameters
      enum ActionType {
         NONE = 0,
         STAND,
         WALK,
         TURN_DRIBBLE,
         GETUP_FRONT,
         GETUP_BACK,
         TIP_OVER,
         KICK,
         INITIAL,
         DEAD,
         REF_PICKUP,
         GOALIE_SIT,
         GOALIE_DIVE_RIGHT,
         GOALIE_DIVE_LEFT,
         GOALIE_CENTRE,
         GOALIE_UNCENTRE,
         GOALIE_INITIAL,
         GOALIE_AFTERSIT_INITIAL,
         DEFENDER_CENTRE,
         GOALIE_FAST_SIT,
         MOTION_CALIBRATE,
         STAND_STRAIGHT,
         LINE_UP,
         TEST_ARMS,
         UKEMI_FRONT,
         UKEMI_BACK,
         GOALIE_STAND,
         KICK_IN_BLUE,
         KICK_IN_RED,
         GOAL_KICK_BLUE,
         GOAL_KICK_RED,
         CORNER_KICK_BLUE,
         CORNER_KICK_RED,
         GOAL_BLUE,
         GOAL_RED,
         PUSHING_FREE_BLUE,
         PUSHING_FREE_RED,
         FULLTIME,
         SIT,
         SIT_UNSTIFF,
         NUM_ACTION_TYPES
      };
      ActionType actionType;

      // Walk/Kick Parameters
      int forward; // How far forward (negative for backwards)  (mm)
      int left;  // How far to the left (negative for rightwards) (mm)
      float turn; // How much anti-clockwise turn (negative for clockwise) (rad)
      float power; // How much kick power (0.0-1.0)
      float bend;
      float speed;

      enum Foot {
         LEFT = 0,
         RIGHT
      };
      Foot foot;

      bool useShuffle;

      // Set this to true if we want to put our arms back
      bool leftArmBehind;
      bool rightArmBehind;

      bool blocking; // whether we are trying to block a moving ball
      bool extraStableKick; // whether we want to ensure the kick is stable, even if the execution is slow

      /**
       * Constructor for walks and kicks
       * @param at Action Type
       * @param f  How far forward (mm)
       * @param l  How far to the left (mm)
       * @param t  How much counter-clockwise turn (rad)
       * @param p  How much power
       * @param bend  Angle to bend knees (rad)
       * @param s  Speed of walk
       * @param k  Direction to kick (rad)
       * @param ft  Which foot to use
       * @param useShuffle use shuffle (low step height) or not
       * @param leftArmBehind make left arm go behind back
       * @param rightArmBehind make right arm go behind back
       * @param blocking to tell walk gen we need a big side step
       * @param extraStableKick  to tell walk gen we prefer kick stability over execution time
       */
      Body(ActionType at, int f = 0, int l = 0, float t = 0.0, float p = 1.0,
           float bend = 15.0, float s = 1.0, Foot ft = LEFT,
           bool useShuffle=false, bool leftArmBehind=false, bool rightArmBehind=false,
           bool blocking=false, bool extraStableKick=false)
         : actionType(at),
           forward(f),
           left(l),
           turn(t),
           power(p),
           bend(bend),
           speed(s),
           foot(ft),
           useShuffle(useShuffle),
           leftArmBehind(leftArmBehind),
           rightArmBehind(rightArmBehind),
           blocking(blocking),
           extraStableKick(extraStableKick) {}

      /* Boost python makes using default arguements difficult.
       * Define an arguementless constructor to wrap
       */
      Body()
         : actionType(NONE),
           forward(0),
           left(0),
           turn(0),
           power(0),
           bend(0),
           speed(0),
           foot(LEFT),
           useShuffle(false),
           leftArmBehind(false),
           rightArmBehind(false),
           blocking(false),
           extraStableKick(false) {}
   };

   const uint8_t priorities[Body::NUM_ACTION_TYPES] = {
      0, // NONE
      0, // STAND
      0, // WALK
      0, // TURN_DRIBBLE
      3, // GETUP_FRONT
      3, // GETUP_BACK
      3, // TIP_OVER
      0, // KICK
      2, // INITIAL
      1, // DEAD
      1, // REF_PICKUP
      2, // GOALIE_SIT
      3, // GOALIE_DIVE_LEFT
      3, // GOALIE_DIVE_RIGHT
      3, // GOALIE_CENTRE
      2, // GOALIE_UNCENTRE
      0, // GOALIE_INITIAL
      0, // GOALIE_AFTERSIT_INITIAL
      2, // DEFENDER_CENTRE
      2, // GOALIE FAST SIT
      0, // MOTION_CALIBRATE
      0, // STAND_STRAIGHT
      0, // LINE_UP
      0, // TEST_ARMS
      3, // UKEMI_FRONT
      3, // UKEMI_BACK
      1, // GOALIE_STAND
      0, // KICK_IN_BLUE
      0, // KICK_IN_RED
      0, // GOAL_KICK_BLUE
      0, // GOAL_KICK_RED
      0, // CORNER_KICK_BLUE
      0, // CORNER_KICK_RED
      0, // GOAL_BLUE
      0, // GOAL_RED
      0, // PUSHING_FREE_BLUE
      0, // PUSHING_FREE_RED
      0, // FULLTIME
      0, // SIT
      1  // SIT_UNSTIFF
   };

/**
 * Command for controlling the head
 */
   struct Head {
      float yaw;      // LEFT-RIGHT motion. Positive is LEFT
      float pitch;    // UP-DOWN angle. Positive is DOWN
      bool isRelative; // TRUE to add to current head angles [DEFAULT]
      float yawSpeed; // Speed of the yaw [0.0, 1.0]
      float pitchSpeed; // Speed of the pitch [0.0, 1.0]

      /**
       * Constructor
       * @param y Yaw amount (Left is positive) (rad)
       * @param p Pitch amount (Down is positive) (rad)
       * @param r Enable relative adjustment (default). False for absolute
       * @param ys Yaw speed [0.0, 1.0]
       * @param ps Pitch speed [0.0, 1.0]
       */
      Head(float y, float p = 0.0, bool r = true,
           float ys = 1.0, float ps = 1.0) : yaw(y),
                                             pitch(p),
                                             isRelative(r),
                                             yawSpeed(ys),
                                             pitchSpeed(ps) {}

      Head()
         : yaw(0.0),
           pitch(0.0),
           isRelative(true),
           yawSpeed(1.0),
           pitchSpeed(1.0) {}
   };

   struct rgb {
      float red;
      float green;
      float blue;

      explicit rgb(float r = 0, float g = 0, float b = 0) : red(r),
                                                            green(g),
                                                            blue(b) {}
   };

   struct rgbSegments {
      std::array<rgb, 8> segments;

      // Constructor
      rgbSegments(const std::array<rgb, 8>& inputSegments) : segments(inputSegments) {}

      // Default constructor with all LEDs off
      rgbSegments() : segments({rgb(), rgb(), rgb(), rgb(), rgb(), rgb(), rgb(), rgb()}) {}

      rgbSegments(const std::array<float, 24>& inputFloats) {
         if (inputFloats.size() == 24) {
            for (size_t i = 0; i < 8; ++i) {
               segments[i] = rgb(inputFloats[i * 3], inputFloats[i * 3 + 1], inputFloats[i * 3 + 2]);
            }
         } else {
            // Handle error or throw an exception
            // For simplicity, here's a print statement
            std::cerr << "Input array must have exactly 24 float values" << std::endl;
         }
      }

      // Constructor taking a Python list
      rgbSegments(const boost::python::list& inputList) {
         if (boost::python::len(inputList) == 24) {
               for (size_t i = 0; i < 8; ++i) {
                  float r = boost::python::extract<float>(inputList[i * 3]);
                  float g = boost::python::extract<float>(inputList[i * 3 + 1]);
                  float b = boost::python::extract<float>(inputList[i * 3 + 2]);
                  segments[i] = rgb(r, g, b);
               }
         } else {
               // Handle error or throw an exception
               std::cerr << "Input list must have exactly 24 float values" << std::endl;
         }
      }

      // Legacy constructor
      rgbSegments(const rgb& rgb) : segments({
         rgb,
         rgb,
         rgb,
         rgb,
         rgb,
         rgb,
         rgb,
         rgb,
      }) {}
   };

   struct LED {

      // NOTE: leftEar is not used and is handled entirely in libagent/LoLA*
      uint16_t leftEar; // Number of left ear segments lit [10-bit field]
      uint16_t rightEar; // Number of right ear segments lit [10-bit field]
      rgbSegments leftEye;     // Colour of left eye (default: white)
      rgbSegments rightEye;    // Colour of right eye (default: white)
      rgb chestButton; // Colour of chest button (default: white)
      rgb leftFoot;    // Colour of left foot (default: off)
      rgb rightFoot;   // Colour of right foot (default: off)

      LED(rgbSegments leye, rgbSegments reye = rgbSegments(), rgb cb = rgb(1.0, 1.0, 1.0),
          rgb lf = rgb(), rgb rf = rgb()) : leftEar(0x3FF),
                                            rightEar(0x3FF),
                                            leftEye(leye),
                                            rightEye(reye),
                                            chestButton(cb),
                                            leftFoot(lf),
                                            rightFoot(rf) {}

      LED()
            : leftEar(0x3FF),
              rightEar(0x3FF),
              leftEye(rgb(1.0, 1.0, 1.0)),
              rightEye(rgb(1.0, 1.0, 1.0)),
              chestButton(rgb(1.0, 1.0, 1.0)),
              leftFoot(rgb()),
              rightFoot(rgb()) {}
   };

   enum Stiffen
   {
      NONE = 0,
      STIFFEN
   };



/**
 * Wrapper for the other action commands, makes it easier to pass them around
 */
   struct All {
      Head head;
      Body body;
      LED leds;
      Stiffen stiffen;
      float ballAge;

      All() : head(), body(Body::NONE), leds(), stiffen(NONE), ballAge(-1)
      { }

      All(Head h, Body b, LED l, float s, Stiffen stf, float age) {
         head = h;
         body = b;
         leds = l;
         stiffen = stf;
         ballAge = age;
      }
   };

//  These classes support stream output for debugging
   static inline bool operator==(const rgb &a, const rgb &b) {
      return (a.red == b.red) && (a.green == b.green) && (a.blue == b.blue);
   }

   static inline bool operator!=(const rgb &a, const rgb &b) {
      return !(a == b);
   }

   static inline std::ostream & operator<<(std::ostream &out, const rgb &a) {
      out << '{' << a.red << ", " << a.green << ", " << a.blue << '}';
      return out;
   }

   static inline std::ostream & operator<<(std::ostream &out, const rgbSegments &a) {
    out << '[';
    for (size_t i = 0; i < a.segments.size(); ++i) {
        out << a.segments[i];
        if (i < a.segments.size() - 1) {
            out << ", ";
        }
    }
    out << ']';
    return out;
}

   static inline std::ostream & operator<<(std::ostream &out, const Head &a) {
      out << '[' << a.yaw << ", " << a.pitch << ", " << a.isRelative << ']';
      return out;
   }

   static inline std::ostream & operator<<(std::ostream &out, const Body &a) {
      out << '[' << a.actionType << ", " << a.forward << ", " << a.left
      << ", " << a.turn << "," << a.power << ']';
      return out;
   }

   static inline std::ostream & operator<<(std::ostream &out, const LED &a) {
      out << '[' << a.leftEar << ", " << a.rightEar << ", " << a.leftEye << ", "
      << a.rightEye << "," << a.chestButton << ","
      << a.leftFoot << "," << a.rightFoot << ']';
      return out;
   }

};  // namespace ActionCommand
