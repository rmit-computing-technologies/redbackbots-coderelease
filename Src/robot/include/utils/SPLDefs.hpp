#pragma once

/** The field coordinate system in mm and radians (rad)
 *  X -- is along the length of the field, +ve towards opponent's goal
 *  Y -- is along the width of the field, +ve towards the left hand side
 *  0 rad -- facing straight towards opponent's goal at origin
 *  radians are calculated counter clock-wise
 *  NOTE: we use -PI, not PI for 180 degrees
 *  NOTE: unless explicitly specified all dimensions includes line width
 */
constexpr int FIELD_LINE_WIDTH = 50;
constexpr int ROBOTS_PER_TEAM = 6;
constexpr bool USE_COMPETITION_FIELD = true;
#define DECLARE_CUSTOM_CONST(name, officialVal, customVal) constexpr float (name) = USE_COMPETITION_FIELD ? (officialVal) : (customVal)

// Competition sizes
/** Field line dimensions */
DECLARE_CUSTOM_CONST(FIELD_LENGTH, 9000, 5620); // 9000 official
DECLARE_CUSTOM_CONST(FIELD_WIDTH, 6000, 3720); // 6000 official
/** How much space is between the outer line and the edge of the green court */
DECLARE_CUSTOM_CONST(FIELD_LENGTH_OFFSET, 700, 432);
DECLARE_CUSTOM_CONST(FIELD_WIDTH_OFFSET, 700, 640);
/** Goal box */
DECLARE_CUSTOM_CONST(GOAL_BOX_LENGTH, 600, 372); // 600
DECLARE_CUSTOM_CONST(GOAL_BOX_WIDTH, 2200, 1364); // 2200
/** Penalty Box */
DECLARE_CUSTOM_CONST(PENALTY_BOX_LENGTH, 1650, 1023);
DECLARE_CUSTOM_CONST(PENALTY_BOX_WIDTH, 4000, 2480);
/** Penalty Cross */
// this is 100 for competition and for our small field
constexpr float PENALTY_CROSS_DIMENSIONS = 100;/* i.e. dimensions of square fitted around it */
DECLARE_CUSTOM_CONST(DIST_GOAL_LINE_TO_PENALTY_CROSS,
                     1300, 806); /* to middle of the closest penalty cross (official 1300) */
constexpr float PENALTY_CROSS_ABS_X = (FIELD_LENGTH / 2 -
                                       DIST_GOAL_LINE_TO_PENALTY_CROSS); /* position relative to centre of field */
/** Center Circle */
DECLARE_CUSTOM_CONST(CENTER_CIRCLE_DIAMETER, 1500, 930);

/** Goal Posts */
DECLARE_CUSTOM_CONST(GOAL_POST_DIAMETER, 100, 100);
constexpr float GOAL_BAR_DIAMETER = 100;  // Double check this once field is built
constexpr float GOAL_POST_HEIGHT = 800; // Measured from the bottom of the crossbar to the ground

constexpr float GOAL_SUPPORT_DIAMETER = 46;
DECLARE_CUSTOM_CONST(GOAL_WIDTH, 1600, 1600); /* top view end-to-end from middle of goal posts */
DECLARE_CUSTOM_CONST(GOAL_DEPTH,
                     500, 500); /* Measured from the front edge of the crossbar to the centre of the rear bar */
//////////////////////////////////////////////////////////////
// May need to define white support bar dimensions for field lines

constexpr float OFFNAO_FIELD_LENGTH_OFFSET = FIELD_LENGTH_OFFSET;
constexpr float OFFNAO_FIELD_WIDTH_OFFSET = FIELD_WIDTH_OFFSET;

/** Field dimensions including edge offsets */
constexpr float FULL_FIELD_LENGTH = (FIELD_LENGTH + (FIELD_LENGTH_OFFSET * 2));
constexpr float OFFNAO_FULL_FIELD_LENGTH = (FIELD_LENGTH + (OFFNAO_FIELD_LENGTH_OFFSET * 2));
constexpr float FULL_FIELD_WIDTH = (FIELD_WIDTH + (FIELD_WIDTH_OFFSET * 2));
constexpr float OFFNAO_FULL_FIELD_WIDTH = (FIELD_WIDTH + (OFFNAO_FIELD_WIDTH_OFFSET * 2));

/** Ball Dimensions */
constexpr float BALL_RADIUS = 50;

/** Post positions in AbsCoord */
// the front of the goal post lines up with the line (as shown in spl rule book)
constexpr float GOAL_POST_ABS_X = (FIELD_LENGTH / 2.0) - (FIELD_LINE_WIDTH / 2.0) + (GOAL_POST_DIAMETER / 2.0);
constexpr float GOAL_POST_ABS_Y = (GOAL_WIDTH / 2);

/** Goal Free Kick Positions in AbsCoord */
constexpr float GOAL_FREE_KICK_ABS_X = PENALTY_CROSS_ABS_X;
constexpr float GOAL_FREE_KICK_ABS_Y = (GOAL_BOX_WIDTH / 2);

/** Corner Kick Positions in AbsCoord */
constexpr float CORNER_KICK_ABS_X = (FIELD_LENGTH / 2);
constexpr float CORNER_KICK_ABS_Y = (FIELD_WIDTH / 2);
