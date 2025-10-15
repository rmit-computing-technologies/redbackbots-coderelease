from util.Constants import ROBOTS_PER_TEAM
from util.Vector2D import Vector2D
from util.Constants import CENTER_CIRCLE_DIAMETER
from util.Timer import Timer
from util import log
from util import EventComms
import robot

blackboard = None
data = None
active_player_numbers = []
non_substituted_player_numbers = []

player_number_with_last_ball_update = 0
players_timer_since_ball_update = [None for _ in range(ROBOTS_PER_TEAM)]
players_seconds_since_ball_update = [100000.0 for _ in range(ROBOTS_PER_TEAM)]

LEFT_MIDFIELDER_CHARGE_TARGET = Vector2D(2000, 2000)
RIGHT_MIDFIELDER_CHARGE_TARGET = Vector2D(2000, -2000)
LEFT_KICK_OFF_TARGET = LEFT_MIDFIELDER_CHARGE_TARGET.minus(Vector2D(-500, 0))
RIGHT_KICK_OFF_TARGET = RIGHT_MIDFIELDER_CHARGE_TARGET.minus(Vector2D(-500, 0))  # noqa

# Amount to add to best ball score every second until yours is lower OR another lower one is sent
# Higher = faster override of another robot's old ball score
SCORE_TIME_CONST_FACTOR = 200

# Amount that the ball score must be lower than the best ball score to take the ball
# For example, if SCORE_TIME_CONST_FACTOR is 100, this means that this will update every 2 seconds from 
# the BEST ball scorer as they will be 200 points ahead of their old score after 2 seconds of adding 100 to it
# Higher = slower switching for robots with similar ball scores
BALL_SCORE_HYSTERESIS_BAND = 400

BALL_SCORE_UPDATE_EVENT_NAME = "BALL_SCORE_UPDATE"
BALL_SCORE_SEND_TIME = 0.0

GOALIE_PLAYER_NUMBERS = [1,20]


def update_team_status(new_blackboard):
    """
    Updates the TeamStatus.py global variables.

    Callable via `TeamStatus.update_team_status(blackboard)`.

    :param new_blackboard: What to update the globals in this module with.
    :return: None
    """
    global blackboard
    blackboard = new_blackboard

    global data
    data = blackboard.receiver.data

    update_active_player_numbers()

    update_players_seconds_since_ball_update()

    update_player_number_with_last_ball_update()

def update_active_player_numbers():
    """
    Updates the list of active player numbers depending on the state game controller.
        - If game controller is not active, returns player number of current robot.
        - If game controller is active, returns list of players not in penalty.

    :return: None
    """
    global active_player_numbers
    active_player_numbers = []

    if not blackboard.gameController.active:
        active_player_numbers.append(my_player_number())
    else:
        for player_number in range(1, ROBOTS_PER_TEAM + 1):
            if not player_number_is_incapacitated(player_number):
                active_player_numbers.append(player_number)


def update_non_substituted_player_numbers():
    """
    Updates the list of non-substituted player numbers.

    :return: None
    """
    global non_substituted_player_numbers
    non_substituted_player_numbers = []
    for player_number in range(1, ROBOTS_PER_TEAM + 1):
        if not player_number_is_substituted(player_number):
            non_substituted_player_numbers.append(player_number)


def update_player_number_with_last_ball_update():
    """
    Updates the player number with the last ball update.

    :return: None
    """
    global player_number_with_last_ball_update
    for player in active_player_numbers:
        player_index = player_number_to_player_index(player)
        if data[player_index].sharedStateEstimationBundle.haveBallUpdate:
            player_number_with_last_ball_update = player


def update_players_seconds_since_ball_update():
    """
    Updates the seconds since the last ball update for each player.

    :return: None
    """
    global players_timer_since_ball_update
    global players_seconds_since_ball_update
    for player in active_player_numbers:
        player_index = player_number_to_player_index(player)
        if data[player_index].sharedStateEstimationBundle.haveBallUpdate:
            players_timer_since_ball_update[player_index] = Timer()
        if players_timer_since_ball_update[player_index] is not None:
            players_seconds_since_ball_update[player_index] = \
                players_timer_since_ball_update[player_index].elapsed_seconds()


def my_player_number() -> int:
    """
    Gets the current player's number.

    :return: The current player's number.
    """
    return blackboard.gameController.player_number


def is_first_team() -> bool:
    """
    Checks if our team is the first team in the teams array from the game controller.

    :return: True if our team is the first team, False otherwise.
    """
    our_team_num = blackboard.gameController.our_team.teamNumber
    first_team = blackboard.gameController.data.teams[0].teamNumber
    return our_team_num == first_team


def player_number_is_incapacitated(player_number: int) -> bool:
    """
    Checks if a player is incapacitated.

    :param player_number: The player number to check.
    :return: True if the player is incapacitated, False otherwise.
    """
    player_index = player_number_to_player_index(player_number)
    return blackboard.gameController.our_team.players[player_index].penalty


def player_number_is_substituted(player_number: int) -> bool:
    """
    Checks if a player is substituted.

    :param player_number: The player number to check.
    :return: True if the player is substituted, False otherwise.
    """
    player_index = player_number_to_player_index(player_number)
    return blackboard.gameController.our_team.players[player_index].penalty == robot.PENALTY_SUBSTITUTE


def player_number_to_player_index(player_number: int) -> int:
    """
    Converts a player number to a player index.

    :param player_number: The player number to convert.
    :return: The corresponding player index.
    """
    return player_number - 1


def player_index_to_player_number(player_index: int) -> int:
    """
    Converts a player index to a player number.

    :param player_index: The player index to convert.
    :return: The corresponding player number.
    """
    return player_index + 1


def pose_of_player_number(player_number: int) -> tuple:
    """
    Gets the pose of a player.

    :param player_number: The player number to get the pose of.
    :return: A tuple containing the position (Vector2D) and heading (float) of the player.
    """
    player_index = player_number_to_player_index(player_number)
    pose = data[player_index].sharedStateEstimationBundle.robotPos
    return Vector2D(pose.x, pose.y), pose.theta


def player_numbers_playing_ball() -> list:
    """
    Gets the player numbers of players playing the ball.

    :return: A list of player numbers of players playing the ball.
    """
    player_numbers = []
    for player_number in active_player_numbers:
        if player_is_playing_ball(player_number):
            player_numbers.append(player_number)
    return player_numbers


def player_is_playing_ball(player_number: int) -> bool:
    """
    Checks if a player is playing the ball.

    :param player_number: The player number to check.
    :return: True if the player is playing the ball, False otherwise.
    """
    player_index = player_number_to_player_index(player_number)
    return data[player_index].behaviourSharedData.playingBall


def playing_ball_score(player_number: int) -> int:
    """
    Gets the playing ball score of a player.

    :param player_number: The player number to get the score of.
    :return: The playing ball score of the player.
    """
    player_index = player_number_to_player_index(player_number)
    return data[player_index].behaviourSharedData.playingBallScore


def do_i_have_highest_playing_ball_score() -> bool:
    """
    Checks if the current player has the highest playing ball score.

    :return: True if the current player has the highest playing ball score, False otherwise.
    """
    players_playing_ball = player_numbers_playing_ball()
    my_score = playing_ball_score(my_player_number())
    for player_number in players_playing_ball:
        if player_number is not my_player_number():
            player_score = playing_ball_score(player_number)
            if player_score > my_score:
                return False
    return True


def other_players_near_ball() -> bool:
    """
    Checks if other players are near the ball.

    :return: True if other players are near the ball, False otherwise.
    """
    for player_number in active_player_numbers:
        if player_number is not my_player_number():
            ball_dist = get_teammate_pos(player_number).minus(teammate_ego_ball(player_number)).length()
            if ball_dist < 500:
                return True
    return False


def player_numbers_assisting() -> list:
    """
    Gets the player numbers of players assisting.

    :return: A list of player numbers of players assisting.
    """
    player_numbers = []
    for player_number in active_player_numbers:
        if player_is_assisting(player_number):
            player_numbers.append(player_number)
    return player_numbers


def player_is_assisting(player_number: int) -> bool:
    """
    Checks if a player is assisting.

    :param player_number: The player number to check.
    :return: True if the player is assisting, False otherwise.
    """
    player_index = player_number_to_player_index(player_number)
    return data[player_index].behaviourSharedData.isAssisting


def assistance_is_needed() -> bool:
    """
    Checks if assistance is needed.

    :return: True if assistance is needed, False otherwise.
    """
    for player_number in player_numbers_playing_ball():
        if player_number_needs_assistance(player_number):
            return True
    return False


def player_number_seconds_since_last_kicked(player_number: int) -> int:
    """
    Gets the seconds since the last kick of a player.

    :param player_number: The player number to get the seconds of.
    :return: The seconds since the last kick of the player.
    """
    player_index = player_number_to_player_index(player_number)
    return data[player_index].behaviourSharedData.secondsSinceLastKick


def player_number_needs_assistance(player_number: int) -> bool:
    """
    Checks if a player needs assistance.

    :param player_number: The player number to check.
    :return: True if the player needs assistance, False otherwise.
    """
    player_index = player_number_to_player_index(player_number)
    return data[player_index].behaviourSharedData.needAssistance


def player_number_that_kicked_ball_last() -> int:
    """
    Gets the player number that kicked the ball last.

    :return: The player number that kicked the ball last.
    """
    min_seconds_since_last_kick = None
    player_number_that_kicked_ball_last = None
    for player_number in active_player_numbers:
        if (min_seconds_since_last_kick is None or
                player_number_that_kicked_ball_last is None) and (player_number_seconds_since_last_kicked(player_number) >= 0):
            min_seconds_since_last_kick = player_number_seconds_since_last_kicked(player_number)
            player_number_that_kicked_ball_last = player_number
        else:
            if min_seconds_since_last_kick > player_number_seconds_since_last_kicked(player_number) and (player_number_seconds_since_last_kicked(player_number) >= 0):
                min_seconds_since_last_kick = player_number_seconds_since_last_kicked(player_number)
                player_number_that_kicked_ball_last = player_number
    if player_number_that_kicked_ball_last is None:
        return 0
    return player_number_that_kicked_ball_last


def i_kicked_the_ball_last() -> bool:
    """
    Checks if the current player kicked the ball last.

    :return: True if the current player kicked the ball last, False otherwise.
    """
    return player_number_that_kicked_ball_last() is my_player_number()


def i_saw_ball_last() -> bool:
    """
    Checks if the current player saw the ball last.

    :return: True if the current player saw the ball last, False otherwise.
    """
    return player_number_with_last_ball_update is my_player_number()

def get_player_number_with_last_ball_update() -> int:
    """
    Gets the player number with the last ball update.

    :return: The player number with the last ball update.
    """
    return player_number_with_last_ball_update


def get_teammate_seconds_since_last_ball_update(player_number: int) -> float:
    """
    Gets the seconds since the last ball update for a teammate.

    :param player_number: The player number of the teammate.
    :return: The seconds since the last ball update for the teammate.
    """
    index = player_number_to_player_index(player_number)
    return players_seconds_since_ball_update[index]


def get_teammate_pos(player_number: int) -> Vector2D:
    """
    Gets the position of a teammate.

    :param player_number: The player number of the teammate.
    :return: The position of the teammate as a Vector2D.
    """
    return pose_of_player_number(player_number)[0]


def get_teammate_heading(player_number: int) -> float:
    """
    Gets the heading of a teammate.

    :param player_number: The player number of the teammate.
    :return: The heading of the teammate.
    """
    return pose_of_player_number(player_number)[1]


def get_active_player_numbers() -> list:
    """
    Gets the list of active player numbers depending on the state of the game.
        - If game state is INITIAL, returns player number of current robot.
        - If game state is NOT INITIAL, returns list of players not in penalty.

    :return: A list of active player numbers.
    """
    update_active_player_numbers()
    return active_player_numbers


def get_non_substituted_player_numbers() -> list:
    """
    Gets the list of non-substituted player numbers.

    :return: A list of non-substituted player numbers.
    """
    update_non_substituted_player_numbers()
    return non_substituted_player_numbers


def get_kick_off_target() -> tuple:
    """
    Gets the kick-off target and the player number.

    :return: A tuple containing the kick-off target (Vector2D) and the player number.
    """
    prefer_left_kick_off = False
    if prefer_left_kick_off:
        players = active_player_numbers[:]  # prefer left
    else:
        players = active_player_numbers[::-1]  # prefer right

    for player in players:
        player_pos = get_teammate_pos(player)

        if player_pos.y < -1000 and abs(player_pos.x) < 1000:
            return RIGHT_KICK_OFF_TARGET, player

        if player_pos.y > 1000 and abs(player_pos.x) < 1000:
            return LEFT_KICK_OFF_TARGET, player

    return None, None


def check_teammate_already_kick_off() -> bool:
    """
    Checks if a teammate has already kicked off.

    :return: True if a teammate has already kicked off, False otherwise.
    """
    for player in active_player_numbers:
        player_index = player_number_to_player_index(player)
        if data[player_index].behaviourSharedData.isKickedOff:
            return True
    return False


def teammate_is_near_centre_circle() -> bool:
    """
    Checks if a teammate is near the centre circle.

    :return: True if a teammate is near the centre circle, False otherwise.
    """
    for player_number in active_player_numbers:
        pos = get_teammate_pos(player_number)
        if pos.length() < CENTER_CIRCLE_DIAMETER / 2 + 200:
            return True
    return False


def player_role(player_number: int) -> str:
    """
    Gets the role of a player.

    :param player_number: The player number to get the role of.
    :return: The role of the player.
    """
    player_index = player_number_to_player_index(player_number)
    return data[player_index].behaviourSharedData.role


def kick_notified() -> bool:
    """
    Checks if someone on the team is notifying about a kick.

    :return: True if someone on the team is notifying about a kick, False otherwise.
    """
    for player in active_player_numbers:
        index = player_number_to_player_index(player)
        if data[index].behaviourSharedData.kickNotification:
            return True
    return False


def teammate_ego_ball(player_number: int) -> Vector2D:
    """
    Gets the ball position relative to the field for a teammate.

    :param player_number: The player number of the teammate.
    :return: The ball position relative to the field as a Vector2D.
    """
    index = player_number_to_player_index(player_number)
    pos = blackboard.receiver.data[index].ballPosAbs
    return Vector2D(pos.x, pos.y)


def player_one_is_field_player() -> bool:
    """
    Checks if player one is a field player.

    :return: True if player one is a field player, False otherwise.
    """
    return blackboard.behaviour.positioning == "PositioningAgainstDribbleTeam"

def robot_can_score() -> bool:
    """
    Check if enough players have touched the ball for the robot to score a goal

    :return: true if enough players have touched the ball
    """
    return blackboard.stateEstimation.canScore

def get_time_scaled_ball_score(player_number: int) -> int:
    """
    Gets the time-scaled ball score of a player.

    :param player_number: The player number to get the score of.
    :return: The time-scaled ball score of the player.
    """
    time_since_last = EventComms.seconds_since_received_by_player_event(player_number, "BALL_SCORE_UPDATE")
    player_ballscore = EventComms.received_data_by_player_event(player_number, "BALL_SCORE_UPDATE")

    log.debug(f"Player {player_number} ball score: {player_ballscore}, time since last: {time_since_last}")

    if player_ballscore is not None and time_since_last is not None:
        if player_ballscore != -1 and time_since_last != 0.0:
            return max((time_since_last) * SCORE_TIME_CONST_FACTOR + player_ballscore, 0)

    return -1

def time_scaled_ball_score_best(my_score: int) -> bool:
    """
    Checks if the current player's time-scaled ball score is the best.
    
    This function also handles sending a message (raising an event) under two conditions:
      1) Our actual ball score is better (lower) than anyone else's time-scaled score,
         but our own time-scaled score is not already the best.
      2) Our time-scaled score is already the best, but our actual ball score is worse
         than that best time-scaled score.

    :param my_score: The current player's actual ball score.
    :return: True if our time-scaled ball score is (or will be) the best, else False.
    """

    def _raise_ball_score_event(score: int, time: float):
        if not EventComms.is_raised(BALL_SCORE_UPDATE_EVENT_NAME):
            EventComms.raise_event(BALL_SCORE_UPDATE_EVENT_NAME, score, time)
            return True
        return False

    # Gather time-scaled scores
    player_scores = {}
    for player in active_player_numbers:
        scaled = get_time_scaled_ball_score(player)
        player_scores[player] = scaled
        log.debug(f"Scaled player {player} score: {scaled}")

    # Find the best (lowest) valid time-scaled score
    best_scaled_player = None
    best_scaled_val = None
    for p, val in player_scores.items():
        if val != -1:
            if best_scaled_val is None or val < best_scaled_val:
                best_scaled_val = val
                best_scaled_player = p

    # If no valid scores were found, default to raising an event for our own score
    if best_scaled_val is None:
        log.debug("No valid scaled ball scores found, raising event for my score.")
        if (_raise_ball_score_event(my_score, BALL_SCORE_SEND_TIME)):
            log.debug(f"Mine {my_player_number()}", say=True)
        return True

    my_scaled_val = get_time_scaled_ball_score(my_player_number())
    log.debug(f"My scaled ball score: {my_scaled_val}")

    # Condition 1: we're not the best scaled, but our actual (+ buffer) is better than the best scaled
    if best_scaled_player != my_player_number() and my_score + BALL_SCORE_HYSTERESIS_BAND < best_scaled_val:
        log.debug("My actual ball score overrides the current best scaled score.")
        if(_raise_ball_score_event(my_score, BALL_SCORE_SEND_TIME)):
            log.debug(f"Mine {my_player_number()}", say=True)
        return True

    # Condition 2: we are the best scaled, but our actual is worse than that best scaled
    if best_scaled_player == my_player_number() and my_score > best_scaled_val + BALL_SCORE_HYSTERESIS_BAND:
        log.debug("My actual ball score is worse than my public one; sending update event.")
        # Simulate what the new best would be if we update our score
        hypothetical_scores = player_scores.copy()
        hypothetical_scores[my_player_number()] = my_score
        # Find the new best after update
        new_best_player = None
        new_best_val = None
        for p, val in hypothetical_scores.items():
            if val != -1:
                if new_best_val is None or val < new_best_val:
                    new_best_val = val
                    new_best_player = p
        
        # Send update
        if (_raise_ball_score_event(my_score, BALL_SCORE_SEND_TIME)):
            # If we will still be the best, just say update
            if new_best_player == my_player_number():
                log.debug(f"Update {my_player_number()}", say=True)
            else:
                log.debug("Yours", say=True)

        if new_best_player == my_player_number():
            return True
        else:
            return False

    # If we're the best scaled and no override was needed, return True
    if best_scaled_player == my_player_number():
        log.debug("I have the best time-scaled ball score.")  # FIXME: This maybe should include buffer?, sometimes there is an instance where both people think they have the best scaled score because they both have slightly different receive times?
        return True

    # Otherwise, someone else has a time-scaled score better than our actual
    log.debug("Another player has a better time-scaled ball score than my actual score.")
    return False
