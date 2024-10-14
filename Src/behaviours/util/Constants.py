import robot
from array import array


FIELD_WIDTH = robot.FIELD_WIDTH
HALF_FIELD_WIDTH = FIELD_WIDTH / 2.0
FIELD_LENGTH = robot.FIELD_LENGTH
HALF_FIELD_LENGTH = FIELD_LENGTH / 2.0
GOAL_BOX_WIDTH = robot.GOAL_BOX_WIDTH
GOAL_BOX_LENGTH = robot.GOAL_BOX_LENGTH
PENALTY_BOX_WIDTH = robot.PENALTY_BOX_WIDTH
PENALTY_BOX_LENGTH = robot.PENALTY_BOX_LENGTH
CENTER_CIRCLE_DIAMETER = robot.CENTER_CIRCLE_DIAMETER
GOAL_WIDTH = robot.GOAL_WIDTH
BALL_RADIUS = robot.BALL_RADIUS
GOAL_POST_ABS_X = robot.GOAL_POST_ABS_X
GOAL_POST_ABS_Y = robot.GOAL_POST_ABS_Y
GOAL_POST_DIAMETER = robot.GOAL_POST_DIAMETER
ROBOTS_PER_TEAM = robot.ROBOTS_PER_TEAM
PENALTY_CROSS_ABS_X = robot.PENALTY_CROSS_ABS_X
PENALTY_CROSS_DISTANCE = robot.DIST_GOAL_LINE_TO_PENALTY_CROSS
PENALTY_AREA_WIDTH = robot.GOAL_BOX_WIDTH
GOAL_FREE_KICK_ABS_X = robot.GOAL_FREE_KICK_ABS_X
GOAL_FREE_KICK_ABS_Y = robot.GOAL_FREE_KICK_ABS_Y
CORNER_KICK_ABS_X = robot.CORNER_KICK_ABS_X
CORNER_KICK_ABS_Y = robot.CORNER_KICK_ABS_Y
DEFENDING_QUARTER = -(FIELD_LENGTH/4)
MIN_DISTANCE_TO_BALL = CENTER_CIRCLE_DIAMETER / 2.0

# Head joint limits
MIN_HEAD_YAW = robot.HeadYaw_Min
MAX_HEAD_YAW = robot.HeadYaw_Max
MIN_HEAD_PITCH = robot.HeadPitch_Min
MAX_HEAD_PITCH = robot.HeadPitch_Max

# Foot geometry, used for dribblin/approaching the ball
TOE_CENTRE_X = 105
HIP_OFFSET = 50

# Field edge distance
FIELD_LENGTH_OFFSET = robot.FIELD_LENGTH_OFFSET
FIELD_WIDTH_OFFSET = robot.FIELD_WIDTH_OFFSET


class LEDColour(object):
    off = 0, 0, 0
    red = 1, 0, 0
    green = 0, 1, 0
    blue = 0, 0, 1
    yellow = 1, 1, 0
    cyan = 0, 1, 1
    magenta = 1, 0, 1
    white = 1, 1, 1
    orange = 1, 0.5, 0
    purple = 0.5, 0, 0.5
    pink = 1, 0.4, 0.7
    dark_green = 0, 0.4, 0.125

    kick_off_defence = 0.7, 0.098, 0.388
    localising = 0.8, 1, 0
    striker = yellow
    defence_angle = white
    supporter = white
    go_to_ball = orange
    denined_striker = cyan

    playing = green
    penalised = red
    set = yellow
    ready = blue
    calibration = magenta
    finished = off
    initial = off
    standby = cyan

class LEDSegments(object):
    off         = robot.rgbSegments(list(LEDColour.off * 8))
    red         = robot.rgbSegments(list(LEDColour.red * 8))
    green       = robot.rgbSegments(list(LEDColour.green * 8))
    blue        = robot.rgbSegments(list(LEDColour.blue * 8))
    yellow      = robot.rgbSegments(list(LEDColour.yellow * 8))
    cyan        = robot.rgbSegments(list(LEDColour.cyan * 8))
    magenta     = robot.rgbSegments(list(LEDColour.magenta * 8))
    white       = robot.rgbSegments(list(LEDColour.white * 8))
    orange      = robot.rgbSegments(list(LEDColour.orange * 8))
    purple      = robot.rgbSegments(list(LEDColour.purple * 8))
    pink        = robot.rgbSegments(list(LEDColour.pink * 8))
    dark_green  = robot.rgbSegments(list(LEDColour.dark_green * 8))

    playing = green
    penalised = red
    set = yellow
    ready = blue
    calibration = magenta
    finished = off
    initial = off
    standby = cyan

# Time you have to wait before you can enter the circle in a defensive kickoff.
KICKOFF_MIN_WAIT = 10 * 1000 * 1000
MAX_TEAM_BALL_UPDATE_TIME = 13
