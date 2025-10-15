from util.Constants import (
    PENALTY_BOX_WIDTH,
    FIELD_LENGTH,
    PENALTY_BOX_LENGTH,
    GOAL_BOX_LENGTH,
    GOAL_BOX_WIDTH,
    GOAL_WIDTH,
    MAX_TEAM_BALL_UPDATE_TIME,
    CLOSE_TO_POSITION_DISTANCE,
    NOT_CLOSE_TO_POSITION_DISTANCE
)
from util.Vector2D import Vector2D
from util.Timer import WallTimer
from util.TeamStatus import get_active_player_numbers
from util import EventComms

# Object caches.
_robotObstacles = None
_ego_ball_pos_rr = None
_ego_ball_pos_rrc = None
_ego_ball_world_pos = None
_team_ball_world_pos = None
_myPose = None
_myPos = None
_ball_lost_count = 10000
_ball_seen_count = 0
blackboard = None
_ball_lost_time = None
_timer_since_last_team_ball_update = None
_last_seen_ego_ball_pos_rrc = None
_ball_seen_buffer = []
_goalie_stop_kick_in_check = False
_look_target = None

MIN_FRAMES_TO_SEE_BALL = 3
MAX_EGO_BALL_LOST_TIME = 3

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

    global _ego_ball_pos_rr
    _ego_ball_pos_rr = blackboard.stateEstimation.ballPosRR

    global _ego_ball_pos_rrc
    _ego_ball_pos_rrc = blackboard.stateEstimation.ballPosRRC

    global _ego_ball_world_pos
    _ego_ball_world_pos = blackboard.stateEstimation.ballPos

    global _team_ball_world_pos
    _team_ball_world_pos = blackboard.stateEstimation.teamBallPos

    global _myPose
    _myPose = blackboard.stateEstimation.robotPos

    global _myPos
    _myPos = Vector2D(_myPose.x, _myPose.y)
    
    global _goalie_stop_kick_in_check

    global _ball_lost_count
    global _ball_seen_count
    if len(blackboard.vision.balls) > 0:
        _ball_lost_count = 0
        _ball_seen_count += 1
    else:
        _ball_lost_count += 1
        _ball_seen_count = 0

    global _ball_lost_time
    if _ball_lost_time is None:
        _ball_lost_time = WallTimer()

    if ego_see_ball():
        _ball_lost_time.restart()

    global _last_seen_ego_ball_pos_rrc
    if len(blackboard.vision.balls) > 0 or \
            _last_seen_ego_ball_pos_rrc is None:
        _last_seen_ego_ball_pos_rrc = blackboard.stateEstimation.ballPosRRC

    global _ball_seen_buffer
    _ball_seen_buffer.append(1 if ego_see_ball() else 0)
    if len(_ball_seen_buffer) > 100:
        _ball_seen_buffer.pop(0)

    global _look_target


def ball_world_pos() -> Vector2D:
    "Vector2D world coordinates of the ball"
    if believe_more_in_team_ball():
        return team_ball_world_pos()
    return ego_ball_world_pos()


def ball_rel_pos() -> Vector2D:
    "Vector2D RRC coordinates of the ball"
    if believe_more_in_team_ball():
        return team_ball_rel_pos()
    return ego_ball_rel_pos()


def ball_world_vel() -> Vector2D:
    "Vector2D world velocity of the ball, in mm/s"
    if believe_more_in_team_ball():
        return team_ball_world_vel()
    else:
        return ego_ball_world_vel()


def ball_rel_vel() -> Vector2D:
    "Vector2D robot relative velocity of the ball, in mm/s"
    ball_vel_rrc = blackboard.stateEstimation.ballVelRRC
    return Vector2D(ball_vel_rrc.x, ball_vel_rrc.y)


def believe_more_in_team_ball(time_padding=1.5) -> bool:
    """
    Whether we believe more in the team ball position,
    depending on which one was more recently updated.

    time_padding accounts for the 1FPS broadcasting rule
    and a slight positive bias towards the egoball
    """
    if ball_lost_time() > time_since_last_team_ball_update() + time_padding:
        return True
    else:
        return False


def we_see_ball() -> bool:
    """
    Returns true if robot can see ball for at least 5 frames (MIN_FRAMES_TO_SEE_BALL), or team ball timer is less than 10 seconds (MAX_TEAM_BALL_UPDATE_TIME)
    """
    if ego_see_ball(MIN_FRAMES_TO_SEE_BALL) or team_see_ball():
        return True
    else:
        return False


def ego_ball_lost(time = MAX_EGO_BALL_LOST_TIME) -> bool:
    """
    Returns true if the ego ball has been lost for 3 seconds
    """
    return ball_lost_time() > time


def team_ball_lost(time = MAX_TEAM_BALL_UPDATE_TIME) -> bool:
    """
    Returns true if the team ball has not been updated for 10 seconds
    (Default: 10 - TeamBall can be updated from 1-9 seconds).
    """
    return time_since_last_team_ball_update() > time


def is_ball_lost(time=None) -> bool:
    """
    Checks if the ball is lost.
        - If active player numbers is over 1, check ego_ball_lost() and team_ball_lost().
        - If only 1 active player, return ego_ball_lost()

    Replacement for HeadAware._ball_lost.is_max() (now removed) as includes updates to teamBall.

    Return True if both ego and team ball are lost, then the ball location is unknown.
    """

    # If the current active players is over 1.
    if len(get_active_player_numbers()) > 1:
        if time is not None:
            return ego_ball_lost(time) and team_ball_lost(time)
        return ego_ball_lost() and team_ball_lost()

    # If there is only one active player.
    return ego_ball_lost()


def set_look_target(target: Vector2D):
    """
    Sets the look target for the robot to look at.
    This is used by the head skill to determine where to 
    start looking to find the ball.
    
    :param target: The target position in world coordinates.
    """
    _look_target = target


def reset_look_target():
    """
    Resets the look target for the robot to look at.
    This is used by the head skill to determine where to 
    start looking to find the ball.
    """
    _look_target = None


def look_target() -> Vector2D:
    """
    Returns the look target for the robot to look at.
    This is used by the head skill to determine where to 
    start looking to find the ball.
    
    :return: The target position in world coordinates.
    """
    # global _look_target
    return _look_target


def ego_ball_distance():
    return blackboard.stateEstimation.ballPosRR.distance


def ego_ball_heading():
    return blackboard.stateEstimation.ballPosRR.heading


def team_ball_distance():
    return team_ball_world_pos().minus(_myPos).length()


def team_ball_heading():
    return team_ball_world_pos().minus(_myPos).rotate(-myHeading()).heading()


def ego_ball_world_vel():
    egoBallVel = blackboard.stateEstimation.ballVel
    return Vector2D(egoBallVel.x, egoBallVel.y)


def team_ball_world_vel():
    teamBallVel = blackboard.stateEstimation.teamBallVel
    return Vector2D(teamBallVel.x, teamBallVel.y)


def ego_ball_rel_pos():
    egoBallPosRRC = blackboard.stateEstimation.ballPosRRC
    return Vector2D(egoBallPosRRC.x, egoBallPosRRC.y)


def team_ball_rel_pos():
    vec = Vector2D(team_ball_distance(), 0).rotated(team_ball_heading())
    return Vector2D(vec.x, vec.y)


def ego_ball_world_pos():
    return Vector2D(_ego_ball_world_pos.x, _ego_ball_world_pos.y)


def team_ball_world_pos():
    return Vector2D(_team_ball_world_pos.x, _team_ball_world_pos.y)


def ball_distance() -> float:
    "Returns the Euclidian distance to the ball"
    if believe_more_in_team_ball():
        return team_ball_distance()
    else:
        return blackboard.stateEstimation.ballPosRR.distance


def ball_heading():
    if believe_more_in_team_ball():
        return team_ball_heading()
    else:
        return blackboard.stateEstimation.ballPosRR.heading


def ball_lost_time():
    return _ball_lost_time.elapsedSeconds()


def myPose():
    return _myPose


def myPos() -> Vector2D:
    "Vector2D robot world coordinates"
    return _myPos


def myHeading() -> float:
    "The robot world relative heading, in radians"
    return _myPose.theta


def ego_see_ball(frames=1) -> bool:
    "Boolean of whether the robot can currently see the ball"
    return _ball_seen_count >= frames


def team_see_ball() -> bool:
    return time_since_last_team_ball_update() < MAX_TEAM_BALL_UPDATE_TIME

def robotObstaclesList():
    "Robot Obstacles"
    # Convert blackboard array to an easier to use list
    global _robotObstacles
    if _robotObstacles is not None:
        return _robotObstacles
    _robotObstacles = []
    for i in range(len(blackboard.stateEstimation.robotObstacles)):
        _robotObstacles.append(blackboard.stateEstimation.robotObstacles[i])
    return _robotObstacles


def ball_lost_frames():
    return _ball_lost_count


def myPosUncertainty():
    return blackboard.stateEstimation.robotPosUncertainty


def myHeadingUncertainty():
    return blackboard.stateEstimation.robotHeadingUncertainty


def team_ball_pos_uncertainty():
    return blackboard.stateEstimation.teamBallPosUncertainty


def myPoseHypothesesCount():
    return len(blackboard.stateEstimation.allRobotPos)


def usingGameSkill():
    return blackboard.behaviour.skill == "Game"


def time_since_last_team_ball_update():
    """
    Returns the time since the last team ball update in seconds.
    If the team ball has never been updated, returns a large number.
    """
    time_since_last = EventComms.time_since_received_by_name("TEAM_BALL_UPDATE")
    return time_since_last if time_since_last is not None else float('inf')

def last_seen_ego_ball_pos_rrc() -> Vector2D:
    "RRC Vector of the ego ball from the last frame where the ball was seen"
    return Vector2D(_last_seen_ego_ball_pos_rrc.x, _last_seen_ego_ball_pos_rrc.y)


def num_balls_seen(frames = 30) -> int:
    """
    Number of balls seen in the last "frames" frames

    :param frames: Number of frames to consider
    :return: Number of balls seen in the last "frames" frames
    """
    return sum(_ball_seen_buffer[-frames:])

def proportion_of_balls_seen(frames = 30) -> float:
    """
    Proportion of balls seen in the last "frames" frames

    :param frames: Number of frames to consider
    :return: Proportion from 0 to 1 of balls seen in the last "frames" frames
    """
    return num_balls_seen(frames) / frames

def ball_seen_in_last_n_frames(frames) -> bool:
    """
    Whether we have seen a ball in the last n frames

    :param frames: Number of frames to consider
    :return: True if we have seen a ball in the last n frames
    """
    return sum(_ball_seen_buffer[-frames:]) >= 1

def _is_in_box(position):
    in_penalty_box = (
        -PENALTY_BOX_WIDTH / 2 < position.y < PENALTY_BOX_WIDTH / 2
        and abs(position.x) > FIELD_LENGTH / 2 - PENALTY_BOX_LENGTH
    )
    in_goal_box = (
        -GOAL_BOX_WIDTH / 2 < position.y < GOAL_BOX_WIDTH / 2
        and abs(position.x) > FIELD_LENGTH / 2 - GOAL_BOX_LENGTH
    )
    return in_penalty_box, in_goal_box

def _is_in_our_box(position):
    in_penalty_box = (
        -PENALTY_BOX_WIDTH / 2 < position.y < PENALTY_BOX_WIDTH / 2
        and position.x < -(FIELD_LENGTH / 2 - PENALTY_BOX_LENGTH)
    )
    in_goal_box = (
        -GOAL_BOX_WIDTH / 2 < position.y < GOAL_BOX_WIDTH / 2
        and position.x < -(FIELD_LENGTH / 2 - GOAL_BOX_LENGTH)
    )
    return in_penalty_box, in_goal_box

def is_ball_in_box() -> tuple:
    """
    is the ball in any goal box or penalty box? Returns a tuple with these answers
    returns (in_penalty_box, in_goal_box)
    """
    return _is_in_box(ball_world_pos())

def is_ball_in_our_box() -> tuple:
    """
    is the ball in our goal box or penalty box? Returns a tuple with these answers
    returns (in_penalty_box, in_goal_box)
    """
    return _is_in_our_box(ball_world_pos())

def is_robot_in_box() -> tuple:
    """
    Returns a tuple of booleans indicating whether the robot is in the penalty box and/or the goal box.
    The first boolean indicates if the robot is in the penalty box, and the second boolean indicates
    if the robot is in the goal box.
    """

    return _is_in_box(myPos())

def is_robot_in_our_box() -> tuple:
    """
    Returns a tuple of booleans indicating whether the robot is in our penalty box and/or the goal box.
    The first boolean indicates if the robot is in our penalty box, and the second boolean indicates
    if the robot is in our goal box.
    """

    return _is_in_our_box(myPos())

def is_ball_in_our_half(time=0) -> bool:
    """
    Is the ball in our half of the field
    
    Returns:
        bool: True if the robot can see the ball and it's closer in our half. 
              Otherwise use team ball posistion after time, (seconds).
    """
    if ego_see_ball(time):
        return ball_world_pos().x < 0
    else:
        return team_ball_world_pos().x < 0

def is_ball_in_goal_box_and_between_goal_posts() -> bool:
    """
    is the ball in the goal box or penalty box? Returns a boolean
    """
    BUFFER = 200
    HALF_GOAL_WIDTH_WITH_BUFFER = (GOAL_WIDTH / 2)-BUFFER
    in_goal_box_and_between_goal_posts = (
        -HALF_GOAL_WIDTH_WITH_BUFFER < ball_world_pos().y < HALF_GOAL_WIDTH_WITH_BUFFER
        and abs(ball_world_pos().x) > FIELD_LENGTH / 2 - GOAL_BOX_LENGTH
    )
    return in_goal_box_and_between_goal_posts

def close_to_position(position: Vector2D, distance=CLOSE_TO_POSITION_DISTANCE):
    """
        Description:
        This function returns True if the robot is close to the desired position.
        If there is a distance specified, it will use that distance instead of the default.
    """
    return position.minus(myPos()).length2() < distance ** 2

def not_close_to_position(position: Vector2D):
    """
        Description:
        This function returns True if the robot is not close to the desired position.
    """
    return position.minus(myPos()).length2() > NOT_CLOSE_TO_POSITION_DISTANCE ** 2

def get_goalie_stop_kick_in_check() -> bool:
    """
    Returns whether the goalie should stop checking for kick-ins.
    This is used to prevent the goalie from checking for kick-ins when they are not needed.
    """
    return _goalie_stop_kick_in_check

def set_goalie_stop_kick_in_check(value: bool):
    """
    Sets whether the goalie should stop checking for kick-ins.
    This is used to prevent the goalie from checking for kick-ins when they are not needed.
    """
    global _goalie_stop_kick_in_check
    _goalie_stop_kick_in_check = value

def has_touched_ball() -> bool:
    """
    Check if player has touched the ball

    :return: true if player has touched the ball
    """
    return blackboard.stateEstimation.hasTouchedBall