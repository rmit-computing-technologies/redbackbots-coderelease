import robot
from util import MathUtil
from util.Vector2D import Vector2D, angleBetween, makeVector2DCopy
from math import radians, atan2, cos, sin
from util.Global import ball_world_pos, myPos, myHeading
from util.Constants import (
    FIELD_WIDTH,
    FIELD_LENGTH,
    GOAL_POST_DIAMETER,
    GOAL_POST_ABS_X,
    GOAL_POST_ABS_Y,
    HALF_FIELD_LENGTH,
    HALF_FIELD_WIDTH,
    HALF_GOAL_BOX_WIDTH,
    HALF_PENALTY_BOX_WIDTH,
    PENALTY_BOX_LENGTH,
    PENALTY_CROSS_ABS_X,
    GOAL_BOX_LENGTH
)
from util.MathUtil import clamp, normalisedTheta

blackboard = None

# Variables that can be accessed from other classes
_ball_near_our_goal = False
_ball_in_front_of_enemy_goal = False


# Enemy goal vectors.
ENEMY_GOAL_CENTER = Vector2D(FIELD_LENGTH/2.0, 0)
# +100 offset so angles aren't too sharp near goals
ENEMY_GOAL_BEHIND_CENTER = Vector2D(
    FIELD_LENGTH/2.0 + 100, 0)
ENEMY_GOAL_INNER_LEFT = Vector2D(
    FIELD_LENGTH/2.0,
    GOAL_POST_ABS_Y - (GOAL_POST_DIAMETER/2))
ENEMY_GOAL_INNER_RIGHT = Vector2D(
    FIELD_LENGTH/2.0,
    -GOAL_POST_ABS_Y + (GOAL_POST_DIAMETER/2))
ENEMY_GOAL_OUTER_LEFT = Vector2D(
    FIELD_LENGTH/2.0,
    GOAL_POST_ABS_Y + (GOAL_POST_DIAMETER/2))
ENEMY_GOAL_OUTER_RIGHT = Vector2D(
    FIELD_LENGTH/2.0,
    -GOAL_POST_ABS_Y - (GOAL_POST_DIAMETER/2))
ENEMY_PENALTY_CENTER = Vector2D(PENALTY_CROSS_ABS_X, 0)
ENEMY_LEFT_POST = Vector2D(GOAL_POST_ABS_X, GOAL_POST_ABS_Y)
ENEMY_RIGHT_POST = Vector2D(GOAL_POST_ABS_X, -GOAL_POST_ABS_Y)

# Enemy goal line-intersections
ENEMY_LEFT_GOAL_BOX_CORNER = Vector2D(
    HALF_FIELD_LENGTH - GOAL_BOX_LENGTH,
    -HALF_GOAL_BOX_WIDTH)
ENEMY_RIGHT_GOAL_BOX_CORNER = Vector2D(
    HALF_FIELD_LENGTH - GOAL_BOX_LENGTH,
    HALF_GOAL_BOX_WIDTH)
ENEMY_LEFT_PENALTY_BOX_CORNER = Vector2D(
    HALF_FIELD_LENGTH - PENALTY_BOX_LENGTH,
    -HALF_PENALTY_BOX_WIDTH)
ENEMY_RIGHT_PENALTY_BOX_CORNER = Vector2D(
    HALF_FIELD_LENGTH - PENALTY_BOX_LENGTH,
    HALF_PENALTY_BOX_WIDTH)

# Own goal vectors.
OWN_GOAL_CENTER = Vector2D(-FIELD_LENGTH/2.0, 0)
# +100 offset so angles aren't too sharp near goals
OWN_GOAL_BEHIND_CENTER = Vector2D(
    -FIELD_LENGTH/2.0 - 100, 0)

OUR_GOAL_CENTRE = Vector2D(-FIELD_LENGTH/2, 0)
OUR_GOAL_BEHIND_CENTRE = Vector2D(-FIELD_LENGTH/2 - 100, 0)
OUR_PENALTY_CENTER = Vector2D(-PENALTY_CROSS_ABS_X, 0)
OUR_LEFT_POST = Vector2D(-GOAL_POST_ABS_X, GOAL_POST_ABS_Y)
OUR_RIGHT_POST = Vector2D(-GOAL_POST_ABS_X, -GOAL_POST_ABS_Y)

# Our goal line-intersections
OUR_LEFT_GOAL_BOX_CORNER = Vector2D(
    -(HALF_FIELD_LENGTH - GOAL_BOX_LENGTH),
    HALF_GOAL_BOX_WIDTH)
OUR_RIGHT_GOAL_BOX_CORNER = Vector2D(
    -(HALF_FIELD_LENGTH - GOAL_BOX_LENGTH),
    -HALF_GOAL_BOX_WIDTH)
OUR_LEFT_PENALTY_BOX_CORNER = Vector2D(
    -(HALF_FIELD_LENGTH - PENALTY_BOX_LENGTH),
    HALF_PENALTY_BOX_WIDTH)
OUR_RIGHT_PENALTY_BOX_CORNER = Vector2D(
    -(HALF_FIELD_LENGTH - PENALTY_BOX_LENGTH),
    -HALF_PENALTY_BOX_WIDTH)

OUR_MIDLLE_GOAL_POST_BOX_CORNER = -HALF_FIELD_LENGTH + GOAL_BOX_LENGTH/2.0

# The Corners in the field
ENEMY_RIGHT_CORNER = Vector2D((FIELD_LENGTH / 2), FIELD_WIDTH / 2)
OUR_LEFT_CORNER = Vector2D(-FIELD_LENGTH / 2, FIELD_WIDTH / 2)
ENEMY_LEFT_CORNER = Vector2D((FIELD_LENGTH / 2), (-FIELD_WIDTH / 2))
OUR_RIGHT_CORNER = Vector2D(-FIELD_LENGTH / 2, -FIELD_WIDTH / 2)

# Center line points 
FIELD_CENTER = Vector2D(0, 0)
MIDDLE_TOP = Vector2D(0, HALF_FIELD_WIDTH)
MIDDLE_BOTTOM = Vector2D(0, -HALF_FIELD_WIDTH)

def update_field_geometry(newBlackboard):
    """
    Updates the FieldGeometry.py global variables, i.e. the blackboard.

    Callable via `FieldGeometry.update_field_geometry(blackboard)`.

    :param newBlackboard: What to update the globals in this module with.
    :return: None
    """
    global blackboard
    blackboard = newBlackboard
    update_ball_near_our_goal()
    update_ball_in_front_of_enemy_goal()


def calculateTimeToReachBall(robot_pos, robot_heading):
    opponentGoal = ENEMY_GOAL_CENTER
    interceptPoint = ball_world_pos()
    interceptToGoal = opponentGoal.minus(interceptPoint)
    interceptToGoalHeading = MathUtil.normalisedTheta(
        atan2(interceptToGoal.y, interceptToGoal.x))
    return calculateTimeToReachPose(robot_pos, robot_heading,
                                    interceptPoint, interceptToGoalHeading)


TURN_RATE = radians(60.0)  # radians/second
WALK_RATE = 300.0  # mm/second
CIRCLE_STRAFE_RATE = radians(40.0)  # radians/second


def calculateTimeToReachPose(myPos, myHeading, targetPos, targetHeading=None):
    toTarget = targetPos.minus(myPos)
    toTargetHeading = atan2(toTarget.y, toTarget.x)

    # How far we need to turn to point at the targetPos
    toTargetTurn = abs(MathUtil.normalisedTheta(toTargetHeading - myHeading))

    # The straightline distance to walk to the targetPos
    toTargetDistance = toTarget.length()

    # How far we need to turn once we get to the targetPos so that we are
    # facing the targetHeading
    if targetHeading is None:
        toTargetHeadingTurn = 0.0
    else:
        toTargetHeadingTurn = abs(MathUtil.normalisedTheta(
            toTargetHeading - targetHeading))

    return toTargetTurn/TURN_RATE + toTargetDistance/WALK_RATE + \
        toTargetHeadingTurn/CIRCLE_STRAFE_RATE


def angleToPoint(point, absCoord):
    phi = angleBetween(point, makeVector2DCopy(absCoord))
    return MathUtil.normalisedTheta(phi)


# Whether something is inside our goalbox
def isInOurGoalBox(pos, buffx=0, buffy=0):
    return (pos.x < -robot.FIELD_LENGTH / 2 + robot.GOAL_BOX_LENGTH + buffx and
            abs(pos.y) < robot.GOAL_BOX_WIDTH / 2 + buffy)


# Whether something is in the opponent goalbox
def isInOpponentGoalBox(pos, buffx=0, buffy=0):
    return (pos.x > robot.FIELD_LENGTH/2 - robot.GOAL_BOX_LENGTH - buffx and
            abs(pos.y) < robot.GOAL_BOX_WIDTH/2 + buffy)


def addRrToRobot(robotPos, rx, ry):
    x = robotPos.x + cos(robotPos.theta) * rx - \
        sin(robotPos.theta) * ry
    y = robotPos.y + sin(robotPos.theta) * rx + \
        cos(robotPos.theta) * ry
    return x, y


def globalPointToRobotRelativePoint(globalVector):

    robotPos = myPos()
    robotHeading = myHeading()

    return globalVector.minus(robotPos).rotate(-robotHeading)


# Closes y-value from ball to our goal line between posts
def closest_goal_y():
    return clamp(ball_world_pos().y, -GOAL_POST_ABS_Y, GOAL_POST_ABS_Y)


# Closest point from the ball to our goal line between posts
def closest_our_goal_point():
    return Vector2D(-GOAL_POST_ABS_X, closest_goal_y())


# Closest point from the ball to our goal line between posts
def closest_opponent_goal_point():
    return Vector2D(GOAL_POST_ABS_X, closest_goal_y())


# Update whether ball is near our goal, with a noise margin
def update_ball_near_our_goal():
    dist_ball_to_our_goal = \
        ball_world_pos().distanceTo(closest_our_goal_point())

    global _ball_near_our_goal
    if _ball_near_our_goal:
        if dist_ball_to_our_goal > 1500:
            _ball_near_our_goal = False
    else:
        if dist_ball_to_our_goal < 1100:
            _ball_near_our_goal = True


# Update whether ball is in front of opponent goal, with a noise margin
def update_ball_in_front_of_enemy_goal():
    global _ball_in_front_of_enemy_goal
    if _ball_in_front_of_enemy_goal:
        if ball_world_pos().x < FIELD_LENGTH/2 - GOAL_BOX_LENGTH - 500 or \
                abs(ball_world_pos().y) > GOAL_POST_ABS_Y + 100:
            _ball_in_front_of_enemy_goal = False
    else:
        if ball_world_pos().x > FIELD_LENGTH/2 - GOAL_BOX_LENGTH - 250 and \
                abs(ball_world_pos().y) < GOAL_POST_ABS_Y - 100:
            _ball_in_front_of_enemy_goal = True


def ball_near_our_goal():
    return _ball_near_our_goal


def ball_in_front_of_enemy_goal():
    return _ball_in_front_of_enemy_goal


def in_role_position(pos, north=HALF_FIELD_WIDTH, east=HALF_FIELD_WIDTH, south=-HALF_FIELD_WIDTH, west=-HALF_FIELD_LENGTH):
    if pos.y < north and pos.x < east and pos.y > south and pos.x > west:
        return True
    else:
        return False


def heading_error(pos: Vector2D) -> float:
    """Calculates and returns the heading error between the robot's current orientation and the direction 
        to a field position. The error is normalised to ensure it falls within valid heading boundaries.

    Args:
        pos (Vector2D): The world position of the target.

    Returns: 
        heading_error (float): The heading difference between `pos` heading and myHeading in radians.
    """
    return normalisedTheta(
            pos.minus(myPos()).heading() - myHeading()
            )

# TODO: implement function overload (potentially using `isinstance(Vector2D)` in the above function)
# def heading_error(heading: float) -> float:
#     """Calculates and returns the heading error between the robot's current orientation and the direction
#         of another heading. The error is normalised to ensure it falls within valid heading boundaries.

#     Returns: 
#         heading_error (float): The heading difference between `heading` and myHeading in radians.
#     """
#     return normalisedTheta(heading - myHeading())