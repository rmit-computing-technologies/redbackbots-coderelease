import robot
from util.TeamStatus import my_player_number, is_first_team
from util.Constants import PENALTY_BOX_WIDTH, FIELD_WIDTH, CENTER_CIRCLE_DIAMETER
from util.FieldGeometry import (
    OWN_GOAL_CENTER, ENEMY_GOAL_CENTER, OUR_LEFT_POST, OUR_RIGHT_POST, 
    ENEMY_LEFT_POST, ENEMY_RIGHT_POST, isInOurGoalBox, isInOpponentGoalBox, ballWorldPos, 
    OUR_RIGHT_CORNER, OUR_LEFT_CORNER, ENEMY_LEFT_CORNER, ENEMY_RIGHT_CORNER,
    )

blackboard = None
gamestate = None
prev_gamestate = None  # Useful for detecting gamestate transitions

TESTING_WITH_HALF_FIELD = False


def update_game_status(new_blackboad):
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
    prev_gamestate = gamestate
    gamestate = blackboard.gameController.data.state


# Game State for GameController.
class GameState(object):
    INITIAL = robot.STATE_INITIAL
    READY = robot.STATE_READY
    SET = robot.STATE_SET
    PLAYING = robot.STATE_PLAYING
    FINISHED = robot.STATE_FINISHED


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

def game_state():
    return gamestate


def prev_game_state():
    return prev_gamestate


def game_phase():
    return blackboard.gameController.data.gamePhase


def set_play():
    return blackboard.gameController.data.setPlay


def in_penaltyshoot_phase():
    return game_phase() is GamePhase.GAME_PHASE_PENALTYSHOOT

# Just 'motion in set' penalty
def penalised_motion_in_set():
    return blackboard.gameController.our_team.players[
          my_player_number() - 1].penalty == Penalty.PENALTY_ILLEGAL_MOTION_IN_SET


# Return penalised if not none
def penalised():
    return player_number_is_penalised(my_player_number())


def player_number_is_penalised(player_number):
    return blackboard.gameController.our_team.players[
          player_number - 1].penalty != Penalty.PENALTY_NONE


def our_team_number():
    return blackboard.gameController.our_team.teamNumber


def kicking_team():
    return blackboard.gameController.data.kickingTeam


def we_are_kicking_team():
    return our_team_number() == kicking_team()


def in_goal_kick():
    return set_play() == SetPlay.SET_PLAY_GOAL_KICK


def in_pushing_free_kick():
    return set_play() == SetPlay.SET_PLAY_PUSHING_FREE_KICK


def in_corner_kick():
    return set_play() == SetPlay.SET_PLAY_CORNER_KICK


def in_kick_in():
    return set_play() == SetPlay.SET_PLAY_KICK_IN

def in_penalty_kick():
    return set_play() == SetPlay.SET_PLAY_PENALTY_KICK

def in_initial():
    return game_state() == GameState.INITIAL


def in_ready():
    return game_state() == GameState.READY


def in_set():
    return game_state() == GameState.SET

def in_playing():
    return game_state() == GameState.PLAYING


def in_finished():
    return game_state() == GameState.FINISHED


def secs_remaining():
    return blackboard.gameController.data.secsRemaining


def secondary_time():
    return blackboard.gameController.data.secondaryTime


def secs_till_unpenalised():
    return blackboard.gameController.our_team.players[
          my_player_number() - 1].secsTillUnpenalised


def whistle_detected():
    return blackboard.gameController.whistleDetected

# OWN GOAL is the left side of the field and has a negative x
def own_goal():
    if TESTING_WITH_HALF_FIELD:
        if is_first_team():
            return OWN_GOAL_CENTER

        return ENEMY_GOAL_CENTER
    return OWN_GOAL_CENTER

# ENEMY GOAL is the right side of the field and has a postive x
def enemy_goal():
    if TESTING_WITH_HALF_FIELD:
        if is_first_team():
            return ENEMY_GOAL_CENTER
        
        return OWN_GOAL_CENTER
    return ENEMY_GOAL_CENTER

# TODO: Change this to be called enemy Right Corner
def own_left_corner():
    # If the ball is near our left side then the corner kick should be on the left side.
    return OUR_LEFT_CORNER

# TODO: Change this to be called enemy left corner
def own_right_corner():
    # If the ball is near our right side then the corner kick should be on the right side.
    return OUR_RIGHT_CORNER

def enemy_left_corner():
    # If the ball is near the enemy left side then the corner kick should be on the left side.
    return ENEMY_LEFT_CORNER

def enemy_right_corner():
    # If the ball is near the enemy right side then the corner kick should be on the right side.
    return ENEMY_RIGHT_CORNER

def field_mid_point():
    return FIELD_WIDTH / 2

def own_left_post():
    if TESTING_WITH_HALF_FIELD:
        if is_first_team():
            return OUR_LEFT_POST

        return ENEMY_LEFT_POST
    return OUR_LEFT_POST

def own_right_post():
    if TESTING_WITH_HALF_FIELD:
        if is_first_team():
            return OUR_RIGHT_POST
        return ENEMY_RIGHT_POST
    return OUR_RIGHT_POST

def enemy_left_post():
    if TESTING_WITH_HALF_FIELD:
        if is_first_team():
            return ENEMY_LEFT_POST
        return OUR_LEFT_POST
    return ENEMY_LEFT_POST

def enemy_right_post():
    if TESTING_WITH_HALF_FIELD:
        if is_first_team():
            return ENEMY_RIGHT_POST
            
        return OUR_RIGHT_POST
    return ENEMY_RIGHT_POST
    
def is_in_our_goalbox(pos, buffx = 0, buffy = 0):
    if TESTING_WITH_HALF_FIELD:
        if is_first_team():
            return isInOurGoalBox(pos=pos, buffx=buffx, buffy=buffy)
        
        return isInOpponentGoalBox(pos=pos, buffx=buffx, buffy=buffy)
    return isInOurGoalBox(pos=pos, buffx=buffx, buffy=buffy)

def is_in_enemy_goalbox(pos, buffx = 0, buffy = 0):
    if TESTING_WITH_HALF_FIELD:
        if is_first_team():
            return isInOpponentGoalBox(pos=pos, buffx=buffx, buffy=buffy)
        
        return isInOurGoalBox(pos=pos, buffx=buffx, buffy=buffy)
    return isInOpponentGoalBox(pos=pos, buffx=buffx, buffy=buffy)

def is_in_goal_kicking_area():
    """
    if ball.x is in the attacking half, and ball.y is between the Penalty area width, then we're in the corridor and lets try to kick a goal
    """
    # Need to implemenet for 
    # if TESTING_WITH_HALF_FIELD:
    # OWN_GOAL_POSITIVE = own_goal().x >= 0
    # in_attacking_half = False

    # if OWN_GOAL_POSITIVE:
    #     print("OWN_GOAL_POSITIVE")
    #     in_attacking_half = ballWorldPos().x < 0
    # else:
    #     print("NOT_OWN_GOAL_POSITIVE")
    #     in_attacking_half = ballWorldPos().x > 0
    
    return ((ballWorldPos().y > -(PENALTY_BOX_WIDTH / 2)) and (ballWorldPos().y < (PENALTY_BOX_WIDTH / 2))) and ballWorldPos().x > -(CENTER_CIRCLE_DIAMETER / 2.0)
