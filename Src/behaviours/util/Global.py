from util.Constants import PENALTY_BOX_WIDTH, FIELD_LENGTH, PENALTY_BOX_LENGTH, GOAL_BOX_LENGTH, GOAL_BOX_WIDTH, GOAL_WIDTH, MAX_TEAM_BALL_UPDATE_TIME
from util.Vector2D import Vector2D
from util.Timer import WallTimer
from util import Log
from util.MathUtil import normalisedTheta
# from util.GameStatus import enemy_goal

# Object caches.
_robotObstacles = None
_egoBallPosRR = None
_egoBallPosRRC = None
_egoBallWorldPos = None
_teamBallWorldPos = None
_myPose = None
_myPos = None
_ballLostCount = 10000
_ballSeenCount = 0
blackboard = None
_ballLostTime = None
_timerSinceLastTeamBallUpdate = None
_lastSeenEgoBallPosRRC = None
_ballSeenBuffer = []


def update_global(newBlackboard):
    """
    Updates the Global.py global variables, such as the `_robotObstacles`.

    Callable via `Global.update(blackboard)`.

    :param newBlackboard: What to update the globals in this module with.
    :return: None
    """
    global blackboard
    blackboard = newBlackboard

    global _robotObstacles
    _robotObstacles = None

    global _egoBallPosRR
    _egoBallPosRR = blackboard.stateEstimation.ballPosRR

    global _egoBallPosRRC
    _egoBallPosRRC = blackboard.stateEstimation.ballPosRRC

    global _egoBallWorldPos
    _egoBallWorldPos = blackboard.stateEstimation.ballPos

    global _teamBallWorldPos
    _teamBallWorldPos = blackboard.stateEstimation.teamBallPos

    global _myPose
    _myPose = blackboard.stateEstimation.robotPos

    global _myPos
    _myPos = Vector2D(_myPose.x, _myPose.y)

    global _ballLostCount
    global _ballSeenCount
    if len(blackboard.vision.balls) > 0:
        _ballLostCount = 0
        _ballSeenCount += 1
    else:
        _ballLostCount += 1
        _ballSeenCount = 0

    global _ballLostTime
    if _ballLostTime is None:
        _ballLostTime = WallTimer()

    if canSeeBall():
        _ballLostTime.restart()

    global _timerSinceLastTeamBallUpdate
    if _timerSinceLastTeamBallUpdate is None:
        _timerSinceLastTeamBallUpdate = WallTimer()
    
    if teamBallUpdate():
        _timerSinceLastTeamBallUpdate.restart()

    global _lastSeenEgoBallPosRRC
    if len(blackboard.vision.balls) > 0 or \
            _lastSeenEgoBallPosRRC is None:
        _lastSeenEgoBallPosRRC = blackboard.stateEstimation.ballPosRRC

    global _ballSeenBuffer
    _ballSeenBuffer.append(1 if canSeeBall() else 0)
    if len(_ballSeenBuffer) > 100:
        _ballSeenBuffer.pop(0)


# Vector2D world coordinates of the ball
def ballWorldPos():
    if believeMoreInTeamBallPos():
        return teamBallWorldPos()
    return egoBallWorldPos()

global _previous_if_kicking_right_foot
_previous_if_kicking_right_foot = False

def previous_if_kicking_right_foot():
    return _previous_if_kicking_right_foot

def set_previous_if_kicking_right_foot(kicking_foot):
    _previous_if_kicking_right_foot = kicking_foot

# Vector2D RRC coordinates of the ball.
def ballRelPos():
    if believeMoreInTeamBallPos():
        return teamBallRelPos()
    return egoBallRelPos()


# Vector2D world velocity of the ball, in mm/s
def ballWorldVel():
    if believeMoreInTeamBallPos():
        return teamBallWorldVel()
    else:
        return egoBallWorldVel()


# Vector2D robot relative velocity of the ball, in mm/s
def ballRelVel():
    ballVelRRC = blackboard.stateEstimation.ballVelRRC
    return Vector2D(ballVelRRC.x, ballVelRRC.y)


# Whether we believe more in the team ball position, depending on which one was
# more recently updated.
# time_padding accounts for the 1FPS broadcasting rule and a slight positive
# bias towards the egoball
def believeMoreInTeamBallPos(time_padding=1.5):
    if ballLostTime() > timeSinceLastTeamBallUpdate() + \
            time_padding:
        return True
    else:
        return False


def can_see_ball_or_team_ball_updated():
    """
    Returns true if robot can see ball, or team ball timer is less than 5 seconds
    """
    if canSeeBall() or timeSinceLastTeamBallUpdate() < 5:
        return True
    else:
        return False

def isBallLost(time=MAX_TEAM_BALL_UPDATE_TIME):
    """
    Checks both ballLostTime() and timeSinceLastTeamBallUpdate() timers are greater than given seconds (Default: 13 - TeamBall can be updated from 1-12 seconds).\n
    Replacement for HeadAware._ball_lost.is_max() with updates to teamBall.
    Return True if both timers are above, assuming then the ball location is unknown.
    """
    if ballLostTime() > time and timeSinceLastTeamBallUpdate() > time:
        return True
    return False

def egoBallDistance():
    return blackboard.stateEstimation.ballPosRR.distance


def egoBallHeading():
    return blackboard.stateEstimation.ballPosRR.heading


def teamBallDistance():
    return teamBallWorldPos().minus(_myPos).length()


def teamBallHeading():
    return teamBallWorldPos().minus(_myPos).rotate(-myHeading()).heading()


def egoBallWorldVel():
    egoBallVel = blackboard.stateEstimation.ballVel
    return Vector2D(egoBallVel.x, egoBallVel.y)


def teamBallWorldVel():
    teamBallVel = blackboard.stateEstimation.teamBallVel
    return Vector2D(teamBallVel.x, teamBallVel.y)


def egoBallRelPos():
    egoBallPosRRC = blackboard.stateEstimation.ballPosRRC
    return Vector2D(egoBallPosRRC.x, egoBallPosRRC.y)


def teamBallRelPos():
    vec = Vector2D(teamBallDistance(), 0).rotated(teamBallHeading())
    return Vector2D(vec.x, vec.y)


def egoBallWorldPos():
    return Vector2D(_egoBallWorldPos.x, _egoBallWorldPos.y)


def teamBallWorldPos():
    return Vector2D(_teamBallWorldPos.x, _teamBallWorldPos.y)


# Float. Returns the Euclidian distance to the ball.
def ballDistance():
    if believeMoreInTeamBallPos():
        return teamBallDistance()
    else:
        return blackboard.stateEstimation.ballPosRR.distance


def ballHeading():
    if believeMoreInTeamBallPos():
        return teamBallHeading()
    else:
        return blackboard.stateEstimation.ballPosRR.heading


def ballLostTime():
    return _ballLostTime.elapsedSeconds()


def myPose():
    return _myPose


# Vector2D robot world coordinates
def myPos():
    return _myPos


# Float of the robot world relative heading, in radians.
def myHeading():
    return _myPose.theta


# Boolean of whether the robot can currently see the ball.
def canSeeBall(frames=1):
    return _ballSeenCount >= frames


def teamBallUpdate():
    return blackboard.stateEstimation.lastTeamBallUpdate == 0

# Robot Obstacles.
def robotObstaclesList():
    # Convert blackboard array to an easier to use list
    global _robotObstacles
    if _robotObstacles is not None:
        return _robotObstacles
    _robotObstacles = []
    for i in range(len(blackboard.stateEstimation.robotObstacles)):
        _robotObstacles.append(blackboard.stateEstimation.robotObstacles[i])
    return _robotObstacles


def ballLostFrames():
    return _ballLostCount


def myPosUncertainty():
    return blackboard.stateEstimation.robotPosUncertainty


def myHeadingUncertainty():
    return blackboard.stateEstimation.robotHeadingUncertainty


def egoBallPosUncertainty():
    return blackboard.stateEstimation.egoBallPosUncertainty


def teamBallPosUncertainty():
    return blackboard.stateEstimation.teamBallPosUncertainty


def myPoseHypothesesCount():
    return len(blackboard.stateEstimation.allRobotPos)


def usingGameSkill():
    return blackboard.behaviour.skill == "Game"


def timeSinceLastTeamBallUpdate():
    if _timerSinceLastTeamBallUpdate is not None:
        return _timerSinceLastTeamBallUpdate.elapsedSeconds()
    else:
        return 100000


# RRC Vector of the ego ball from the last frame where the ball was seen
def lastSeenEgoBallPosRRC():
    return Vector2D(_lastSeenEgoBallPosRRC.x, _lastSeenEgoBallPosRRC.y)


def numBallsSeenInLastXFrames(x=30):
    return sum(_ballSeenBuffer[-x:])

def proportionOfBallsSeenInLastXFrames(x=30) -> float:
    return sum(_ballSeenBuffer[-x:]) / x

def is_ball_in_box(extra_buffer = 0):
    """
    is the ball in the goal box or penalty box? Returns a tuple with these answers
    """
    in_penalty_box = -(PENALTY_BOX_WIDTH / 2 + extra_buffer) < ballWorldPos().y < (PENALTY_BOX_WIDTH / 2 + extra_buffer) and abs(ballWorldPos().x) > (FIELD_LENGTH / 2 - PENALTY_BOX_LENGTH - extra_buffer)+ 100
    in_goal_box = -(GOAL_BOX_WIDTH / 2 + extra_buffer) < ballWorldPos().y < (GOAL_BOX_WIDTH / 2 + extra_buffer) and abs(ballWorldPos().x) > (FIELD_LENGTH / 2 - GOAL_BOX_LENGTH - extra_buffer)
    return in_penalty_box, in_goal_box

def is_ball_in_our_box(extra_buffer = 0):
    """
    is the ball in our goal box or penalty box? Returns a tuple with these answers
    """
    in_penalty_box = -(PENALTY_BOX_WIDTH / 2 + extra_buffer) < ballWorldPos().y < (PENALTY_BOX_WIDTH / 2 + extra_buffer) and ballWorldPos().x < -FIELD_LENGTH / 2 + PENALTY_BOX_LENGTH + extra_buffer + 100
    in_goal_box = -(GOAL_BOX_WIDTH / 2 + extra_buffer) < ballWorldPos().y < (GOAL_BOX_WIDTH / 2 + extra_buffer) and ballWorldPos().x < -FIELD_LENGTH / 2 + GOAL_BOX_LENGTH + extra_buffer
    return in_penalty_box, in_goal_box

def is_ball_in_goal_box_and_between_goal_posts():
    """
    is the ball in the goal box or penalty box? Returns a boolean
    """
    BUFFER = 300
    HALF_GOAL_WIDTH_WITH_BUFFER = (GOAL_WIDTH / 2)-BUFFER
    in_goal_box_and_between_goal_posts = -HALF_GOAL_WIDTH_WITH_BUFFER < ballWorldPos().y < HALF_GOAL_WIDTH_WITH_BUFFER and abs(ballWorldPos().x) > FIELD_LENGTH / 2 - GOAL_BOX_LENGTH
    return in_goal_box_and_between_goal_posts
