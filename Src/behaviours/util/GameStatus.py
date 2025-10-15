import robot
import math
from util.Vector2D import Vector2D
from util.MathUtil import normalisedTheta
from util.Constants import PENALTY_BOX_WIDTH, FIELD_WIDTH, GOAL_WIDTH, DEFENDING_QUARTER, GOAL_BOX_LENGTH, FIELD_LENGTH, HALF_CENTER_CIRCLE_DIAMETER
from util.FieldGeometry import (
    OWN_GOAL_CENTER,
    ENEMY_GOAL_CENTER,
    OUR_LEFT_POST,
    OUR_RIGHT_POST,
    ENEMY_LEFT_POST,
    ENEMY_RIGHT_POST,
    isInOurGoalBox,
    isInOpponentGoalBox,
    OUR_RIGHT_CORNER,
    OUR_LEFT_CORNER,
    ENEMY_LEFT_CORNER,
    ENEMY_RIGHT_CORNER,
)

from util.TeamStatus import my_player_number, is_first_team
from util.Global import myHeading, ball_world_pos, myPos
from util import log

blackboard = None
gamestate = None
gc_gamestate = None # Used solely for playing->ready gamestate changes, avoid secondaryTime issue
prev_gamestate = None  # Useful for detecting gamestate transitions
whistle_status = False #True when whistle heard
_whistle_crashed = False
TESTING_WITH_HALF_FIELD = False
_consequtive_whistle_detection = 0

def update_game_status(new_blackboad) -> None:
    """
    Updates the GameStatus.py global blackboard variable.

    Callable via `GameStatus.update_game_status(blackboard)`.

    :param newBlackboard: What to update the globals in this module with.
    :return: None
    """
    global blackboard
    blackboard = new_blackboad

    global prev_gamestate
    global gamestate
    global gc_gamestate
    prev_gamestate = gamestate
    gamestate = blackboard.gameController.gameState
    gc_gamestate = blackboard.gameController.data.state
    
    global _consequtive_whistle_detection
    global _whistle_crashed
    if get_whistle_status() == robot.WhistleDetectionState.isDetected:
        _consequtive_whistle_detection += 1
    else:
        _consequtive_whistle_detection = 0
    
    if _consequtive_whistle_detection > 100:
        _whistle_crashed = True

# Game State for GameController.
class GameState(object):
    INITIAL = robot.STATE_INITIAL
    READY = robot.STATE_READY
    SET = robot.STATE_SET
    PLAYING = robot.STATE_PLAYING
    FINISHED = robot.STATE_FINISHED
    STANDBY = robot.STATE_STANDBY


class GamePhase(object):
    GAME_PHASE_NORMAL = robot.GAME_PHASE_NORMAL
    GAME_PHASE_PENALTYSHOOT = robot.GAME_PHASE_PENALTYSHOOT
    # Note: Could not get GameController2015 to actually send OVERTIME
    #       but tested GAME_PHASE_TIMEOUT
    GAME_PHASE_OVERTIME = robot.GAME_PHASE_OVERTIME
    GAME_PHASE_TIMEOUT = robot.GAME_PHASE_TIMEOUT


class SetPlay(object):
    SET_PLAY_NONE = robot.SET_PLAY_NONE
    SET_PLAY_GOAL_KICK  = robot.SET_PLAY_GOAL_KICK
    SET_PLAY_PUSHING_FREE_KICK = robot.SET_PLAY_PUSHING_FREE_KICK
    SET_PLAY_CORNER_KICK = robot.SET_PLAY_CORNER_KICK
    SET_PLAY_KICK_IN = robot.SET_PLAY_KICK_IN
    SET_PLAY_PENALTY_KICK = robot.SET_PLAY_PENALTY_KICK


class Penalty(object):
    # All penalty states found in RoboCupGameControlData.hpp
    PENALTY_NONE = robot.PENALTY_NONE                                           #0
    PENALTY_ILLEGAL_BALL_CONTACT = robot.PENALTY_SPL_ILLEGAL_BALL_CONTACT       #1
    PENALTY_PLAYER_PUSHING = robot.PENALTY_SPL_PLAYER_PUSHING                   #2
    PENALTY_ILLEGAL_MOTION_IN_SET = robot.PENALTY_SPL_ILLEGAL_MOTION_IN_SET     #3
    PENALTY_INACTIVE_PLAYER = robot.PENALTY_SPL_INACTIVE_PLAYER                 #4
    PENALTY_ILLEGAL_POSITION = robot.PENALTY_SPL_ILLEGAL_POSITION               #5
    PENALTY_LEAVING_THE_FIELD = robot.PENALTY_SPL_LEAVING_THE_FIELD             #6
    PENALTY_REQUEST_FOR_PICKUP = robot.PENALTY_SPL_REQUEST_FOR_PICKUP           #7
    PENALTY_LOCAL_GAME_STUCK = robot.PENALTY_SPL_LOCAL_GAME_STUCK               #8
    PENALTY_ILLEGAL_POSITION_IN_SET = robot.PENALTY_SPL_ILLEGAL_POSITION_IN_SET #9
    PENALTY_PLAYER_STANCE = robot.PENALTY_SPL_PLAYER_STANCE                     #10
    PENALTY_SUBSTITUTE = robot.PENALTY_SUBSTITUTE                               #14
    PENALTY_MANUAL = robot.PENALTY_MANUAL                                       #15


def game_controller_active() -> bool:
    """
    Returns whether or not the robot recieves a packet from GC.

    :return: True if packet from GC received, else False.
    """
    return blackboard.gameController.active


def game_state() -> int:
    """
    Returns the current game state.

    :return: Current game state.
    """
    return gamestate


def prev_game_state() -> int:
    """
    Returns the previous game state.

    :return: Previous game state.
    """
    return prev_gamestate

def gc_game_state() -> int:
    """
    Returns the game state according to GC
    Only for ready skill
    Robots should not make decisions based on this

    :return: GameController game state
    """
    return gc_gamestate


def game_phase() -> int:
    """
    Returns the current game phase.

    :return: Current game phase.
    """
    return blackboard.gameController.data.gamePhase


def set_play() -> int:
    """
    Returns the current set play (Which game state we are in, kick-in, free-kick etc).

    :return: Current set play (kick-in, free-kick etc).
    """
    return blackboard.gameController.data.setPlay


def in_penaltyshoot_phase() -> bool:
    """
    Checks if the game is in the penalty shoot phase.

    :return: True if in penalty shoot phase, else False.
    """
    return game_phase() is GamePhase.GAME_PHASE_PENALTYSHOOT

def penalised_motion_in_set() -> bool:
    """
    Checks if the player is penalised for illegal motion in set.

    :return: True if penalised for illegal motion in set, else False.
    """
    return blackboard.gameController.our_team.players[
          my_player_number() - 1].penalty == Penalty.PENALTY_ILLEGAL_MOTION_IN_SET


def penalised() -> bool:
    """
    Checks if the player is penalised.

    :return: True if penalised, else False.
    """
    return player_number_is_penalised(my_player_number())


def player_number_is_penalised(player_number: int) -> bool:
    """
    Checks if the player with the given number is penalised.

    :param player_number: Player number to check.
    :return: True if the player is penalised, else False.
    """
    return blackboard.gameController.our_team.players[
          player_number - 1].penalty != Penalty.PENALTY_NONE


def our_team_number() -> int:
    """
    Returns the team number of our team.

    :return: Our team number.
    """
    return blackboard.gameController.our_team.teamNumber

def playing_on_left_side() -> bool:
    """
    Returns side of field we are on based on GC

    :return: True if on left side of field, else false.
    """
    return blackboard.gameController.left_team

def playing_on_left_side_dynamic() -> bool:
    """
    Returns side of field we are on based on GC & Current Half Time

    :return: True if on left side of field, else false.
    """
    started_on_left_side = blackboard.gameController.left_team
    if (in_first_half):
        return started_on_left_side
    else:
        return (not(started_on_left_side))


def kicking_team() -> int:
    """
    Returns the team number of the next team to kick off, free kick etc.

    :return: Team number of the next team to kick off, free kick etc.
    """
    return blackboard.gameController.data.kickingTeam


def we_are_kicking_team() -> bool:
    """
    Checks if we are the next team to kick off, free kick etc.

    :return: True if we are the next team to kick off, free kick etc, else False.
    """
    return our_team_number() == kicking_team()


def in_goal_kick() -> bool:
    """
    Checks if we are in the GOAL KICK game state.

    :return: True if in GOAL KICK game state, else False.
    """
    return set_play() == SetPlay.SET_PLAY_GOAL_KICK


def in_pushing_free_kick() -> bool:
    """
    Checks if we are in the PUSHING FREE KICK game state.

    :return: True if in PUSHING FREE KICK game state, else False.
    """
    return set_play() == SetPlay.SET_PLAY_PUSHING_FREE_KICK


def in_corner_kick() -> bool:
    """
    Checks if we are in the CORNER KICK game state.

    :return: True if in CORNER KICK game state, else False.
    """
    return set_play() == SetPlay.SET_PLAY_CORNER_KICK


def in_kick_in() -> bool:
    """
    Checks if we are in the KICK IN game state.

    :return: True if in KICK IN game state, else False.
    """
    return set_play() == SetPlay.SET_PLAY_KICK_IN


def in_penalty_kick() -> bool:
    """
    Checks if we are in the PENALTY KICK game state.

    :return: True if in PENALTY KICK game state, else False.
    """
    return set_play() == SetPlay.SET_PLAY_PENALTY_KICK


def in_initial() -> bool:
    """
    Checks if we are in the INITIAL game state.

    :return: True if in INITIAL game state, else False.
    """
    return game_state() == GameState.INITIAL


def in_ready() -> bool:
    """
    Checks if we are in the READY game state.

    :return: True if in READY game state, else False.
    """
    return game_state() == GameState.READY


def in_set() -> bool:
    """
    Checks if we are in the SET game state.

    :return: True if in SET game state, else False.
    """
    return game_state() == GameState.SET


def in_playing() -> bool:
    """
    Checks if we are in the PLAYING game state.

    :return: True if in PLAYING game state, else False.
    """
    return game_state() == GameState.PLAYING


def in_finished() -> bool:
    """
    Checks if we are in the FINISHED game state.

    :return: True if in FINISHED game state, else False.
    """
    return game_state() == GameState.FINISHED

def in_standby() -> bool:
    """
    Checks if we are in the STANDBY game state.

    :return: True if in STANDBY game state, else False.
    """
    return game_state() == GameState.STANDBY

def in_first_half() -> bool:
    """
    Returns if we are in first half

    :return: true if we are in first half, else false.
    """

    return blackboard.gameController.data.firstHalf

def secs_remaining() -> int:
    """
    Returns the number of seconds remaining in the game.

    :return: Seconds remaining in the game.
    """
    return blackboard.gameController.data.secsRemaining


def secondary_time() -> int:
    """
    Returns the secondary time in the game.

    :return: Secondary time in the game.
    """
    return blackboard.gameController.data.secondaryTime


def secs_till_unpenalised() -> int:
    """
    Returns the number of seconds until the player is unpenalised.

    :return: Seconds until the player is unpenalised.
    """
    return blackboard.gameController.our_team.players[
          my_player_number() - 1].secsTillUnpenalised

def get_whistle_status() -> robot.WhistleDetectionState:
    """
    Returns the whistle detection state.

    :return: Whistle detection state.
    """
    return blackboard.whistle.whistleDetectionState

def whistle_detected() -> bool:
    """
    Checks if a whistle has been detected.

    :return: True if a whistle has been detected, else False.
    """
    if _whistle_crashed:
        robot.WhistleDetectionState.whistleThreadCrashed = True
        log.warning("Whistle Crashed", say=False)
        return False
    else:
        return get_whistle_status() == robot.WhistleDetectionState.isDetected

def testing_with_half_field() -> bool:
    """
    Checks if testing is being done with half field.

    :return: True if testing with half field, else False.
    """
    return TESTING_WITH_HALF_FIELD

def own_goal() -> Vector2D:
    """
    Returns the coordinates of the own goal.

    OWN GOAL is the our side of the field and has a negative x
    (when not TESTING_WITH_HALF_FIELD)

    :return: Vector2D world coordinates of the own goal.
    """
    if TESTING_WITH_HALF_FIELD:
        if is_first_team():
            return OWN_GOAL_CENTER.clone()

        return ENEMY_GOAL_CENTER.clone()
    return OWN_GOAL_CENTER.clone()

def just_outside_own_goal() -> Vector2D:
    """
    Returns the position just outside the own goal.

    Position in the centre of the goal Y,
    but slightly towards the centre of the field for X

    :return: Vector2D world coordinates just outside the own goal.
    """
    if TESTING_WITH_HALF_FIELD and not is_first_team():
        return Vector2D(OWN_GOAL_CENTER.x-(GOAL_BOX_LENGTH/2), OWN_GOAL_CENTER.y)
    else:
        return Vector2D(OWN_GOAL_CENTER.x+(GOAL_BOX_LENGTH/2), OWN_GOAL_CENTER.y)

def enemy_goal() -> Vector2D:
    """
    Returns the coordinates of the enemy goal.

    ENEMY GOAL is the opponents side of the field and has a positive x
    (when not TESTING_WITH_HALF_FIELD)

    :return: Vector2D world coordinates of the enemy goal.
    """
    if TESTING_WITH_HALF_FIELD and not is_first_team():
        return OWN_GOAL_CENTER.clone()
    else:
        return ENEMY_GOAL_CENTER.clone()

def own_left_corner() -> Vector2D:
    """
    Returns the coordinates of the own left corner.

    :return: Vector2D world coordinates of the own left corner.
    """
    return OUR_LEFT_CORNER.clone()

def own_right_corner() -> Vector2D:
    """
    Returns the coordinates of the own right corner.

    :return: Vector2D world coordinates of the own right corner.
    """
    return OUR_RIGHT_CORNER.clone()

def enemy_left_corner() -> Vector2D:
    """
    Returns the coordinates of the enemy left corner.

    :return: Vector2D world coordinates of the enemy left corner.
    """
    return ENEMY_LEFT_CORNER.clone()

def enemy_right_corner() -> Vector2D:
    """
    Returns the coordinates of the enemy right corner.

    :return: Vector2D world coordinates of the enemy right corner.
    """
    return ENEMY_RIGHT_CORNER.clone()

def field_mid_point() -> float:
    """
    Returns the midpoint of the field width.

    :return: Midpoint of the field width.
    """
    return FIELD_WIDTH / 2

def own_left_post() -> Vector2D:
    """
    Returns the coordinates of the own left post.

    :return: Vector2D world coordinates of the own left post.
    """
    if TESTING_WITH_HALF_FIELD:
        if is_first_team():
            return OUR_LEFT_POST.clone()

        return ENEMY_LEFT_POST.clone()
    return OUR_LEFT_POST.clone()

def own_right_post() -> Vector2D:
    """
    Returns the coordinates of the own right post.

    :return: Vector2D world coordinates of the own right post.
    """
    if TESTING_WITH_HALF_FIELD:
        if is_first_team():
            return OUR_RIGHT_POST.clone()
        return ENEMY_RIGHT_POST.clone()
    return OUR_RIGHT_POST.clone()

def enemy_left_post() -> Vector2D:
    """
    Returns the coordinates of the enemy left post.

    :return: Vector2D world coordinates of the enemy left post.
    """
    if TESTING_WITH_HALF_FIELD:
        if is_first_team():
            return ENEMY_LEFT_POST.clone()
        return OUR_LEFT_POST.clone()
    return ENEMY_LEFT_POST.clone()

def enemy_right_post() -> Vector2D:
    """
    Returns the coordinates of the enemy right post.

    :return: Vector2D world coordinates of the enemy right post.
    """
    if TESTING_WITH_HALF_FIELD:
        if is_first_team():
            return ENEMY_RIGHT_POST.clone()

        return OUR_RIGHT_POST.clone()
    return ENEMY_RIGHT_POST.clone()

def is_in_our_goalbox(pos: Vector2D, buffx: float = 0, buffy: float = 0) -> bool:
    """
    Checks if the given position is in our goal box.

    :param pos: Position to check.
    :param buffx: Buffer in x direction.
    :param buffy: Buffer in y direction.
    :return: True if the position is in our goal box, else False.
    """
    if TESTING_WITH_HALF_FIELD:
        if is_first_team():
            return isInOurGoalBox(pos=pos, buffx=buffx, buffy=buffy)

        return isInOpponentGoalBox(pos=pos, buffx=buffx, buffy=buffy)
    return isInOurGoalBox(pos=pos, buffx=buffx, buffy=buffy)

def is_in_enemy_goalbox(pos: Vector2D, buffx: float = 0, buffy: float = 0) -> bool:
    """
    Checks if the given position is in the enemy goal box.

    :param pos: Position to check.
    :param buffx: Buffer in x direction.
    :param buffy: Buffer in y direction.
    :return: True if the position is in the enemy goal box, else False.
    """
    if TESTING_WITH_HALF_FIELD:
        if is_first_team():
            return isInOpponentGoalBox(pos=pos, buffx=buffx, buffy=buffy)

        return isInOurGoalBox(pos=pos, buffx=buffx, buffy=buffy)
    return isInOpponentGoalBox(pos=pos, buffx=buffx, buffy=buffy)

def is_ball_between_goal_posts():
    """
    returns whether the ball is in between the goal posts
    """
    return abs(ball_world_pos().y) < (GOAL_WIDTH / 2.0)

# def is_in_goal_kicking_area() -> bool:
#     """
#     if ball.x is out of our defending quarter, and ball.y is between the Penalty area width
#     then we're in the corridor and lets try to kick a goal

#     :return: True if the ball is in the goal kicking area, else False.
#     """
#     # First check if the ball is close to the sidelines
#     if abs(ball_world_pos().y) > (PENALTY_BOX_WIDTH / 2.0):
#         return False

#     # Then check if the ball is on our (less than) half
#     if TESTING_WITH_HALF_FIELD and not is_first_team():
#         if ball_world_pos().x > (FIELD_LENGTH / 4.0):
#             return False
#     else:
#         if ball_world_pos().x < -(FIELD_LENGTH / 4.0):
#             return False

#     # if we got this far, we should go for a goal!
#     return True

def is_in_goal_kicking_area() -> bool:
    """
    if ball.x is out of our defending quarter, and ball.y is between the Penalty area width
    then we're in the corridor and lets try to kick a goal

    :return: True if the ball is in the goal kicking area, else False.
    """
    # First check if the ball is on our (less than) half
    if TESTING_WITH_HALF_FIELD and not is_first_team():
        if ball_world_pos().x > HALF_CENTER_CIRCLE_DIAMETER:
            return False
    else:
        if ball_world_pos().x < -HALF_CENTER_CIRCLE_DIAMETER:
            return False

    # Then check if the ball is at a bad angle to the goal
    angle = enemy_goal().minus(ball_world_pos()).heading()
    if abs(math.degrees(angle)) > 100/2:
        return False

    # if we got this far, we should go for a goal!
    return True

def our_defending_quarter() -> float:
    """
    Returns the defending quarter of the field.

    :return: Defending quarter of the field.
    """
    if TESTING_WITH_HALF_FIELD:
        if is_first_team():
            return DEFENDING_QUARTER
        return -DEFENDING_QUARTER
    return DEFENDING_QUARTER

def is_ball_in_attacking_half() -> bool:
    """
    Checks if the ball is in the attacking half.

    :return: True if the ball is in the attacking half, else False.
    """
    if TESTING_WITH_HALF_FIELD and not is_first_team():
        return ball_world_pos().x < 0
    else:
        return ball_world_pos().x > 0

def enemy_goal_heading() -> float:
    """
    Provides heading error for facing forward.

    :return: Heading error for facing forward.
    """
    return normalisedTheta(
            enemy_goal().minus(myPos()).heading() - myHeading())

def own_goal_heading() -> float:
    """
    Provides heading error for facing backward.

    :return: Heading error for facing backward.
    """
    return normalisedTheta(
            own_goal().minus(myPos()).heading() - myHeading())

def enemy_goal_heading_backward() -> float:
    """
    Provides heading error for facing directly away from the enemy goal.
    """
    return normalisedTheta(
            enemy_goal().minus(myPos()).heading() - myHeading() + math.pi)
